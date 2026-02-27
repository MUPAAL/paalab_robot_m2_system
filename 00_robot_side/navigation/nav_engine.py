"""
nav_engine — 导航引擎主状态机

NavigationEngine:
    - 接收 IMU 数据（20 Hz）驱动控制循环
    - 接收 RTK GPS 数据（1 Hz）更新滤波器
    - 管理航点序列、GPS 滤波器、控制器
    - 通过 send_velocity_fn 发出速度指令
    - 通过 broadcast_fn（异步协程）广播导航状态

状态枚举：
    NavState: IDLE | NAVIGATING | FINISHED

模式枚举：
    NavMode:    P2P | PURE_PURSUIT
    FilterMode: MOVING_AVG | KALMAN
"""

import asyncio
import logging
import math
import threading
import time
from enum import Enum, auto
from typing import Callable, Coroutine

from config import NAV_GPS_TIMEOUT_S, MAX_LINEAR_VEL, MAX_ANGULAR_VEL
from navigation.geo_utils import haversine_distance, bearing_to_target
from navigation.waypoint import WaypointManager
from navigation.gps_filter import MovingAverageFilter, KalmanFilter
from navigation.controller import P2PController, PurePursuitController

logger = logging.getLogger(__name__)


# ── 枚举 ──────────────────────────────────────────────────
class NavState(Enum):
    IDLE       = auto()
    NAVIGATING = auto()
    FINISHED   = auto()


class NavMode(Enum):
    P2P           = "p2p"
    PURE_PURSUIT  = "pure_pursuit"


class FilterMode(Enum):
    MOVING_AVG = "moving_avg"
    KALMAN     = "kalman"


# ── 导航引擎 ──────────────────────────────────────────────
class NavigationEngine:
    """GPS + IMU 自主路径导航引擎。

    Args:
        send_velocity_fn : Callable[[float, float], None]，发送速度指令
        broadcast_fn     : 异步广播函数 async(dict) -> None
        loop             : asyncio 事件循环（用于跨线程调度广播）
    """

    def __init__(
        self,
        send_velocity_fn: Callable[[float, float], None],
        broadcast_fn: Callable[[dict], Coroutine],
        loop: asyncio.AbstractEventLoop,
    ) -> None:
        self._send_velocity = send_velocity_fn
        self._broadcast     = broadcast_fn
        self._loop          = loop

        self._lock = threading.Lock()

        # 状态
        self._state: NavState   = NavState.IDLE
        self._nav_mode: NavMode       = NavMode.P2P
        self._filter_mode: FilterMode = FilterMode.MOVING_AVG

        # 航点管理
        self._wp_mgr = WaypointManager()

        # 滤波器
        self._ma_filter  = MovingAverageFilter()
        self._kf_filter  = KalmanFilter()

        # 控制器
        self._p2p_ctrl = P2PController()
        self._pp_ctrl  = PurePursuitController()

        # IMU 状态
        self._robot_bearing: float | None = None  # None = 未校准
        self._last_imu_ts:   float = 0.0
        self._last_control_ts: float = 0.0

        # RTK 状态
        self._fix_quality: int   = 0
        self._last_gps_ts: float = 0.0
        self._gps_warning_sent: bool = False

        # 广播节流
        self._broadcast_counter: int = 0

    # ── 公共接口 ──────────────────────────────────────────
    def load_waypoints(self, csv_text: str) -> int:
        """加载 CSV 航点文本，返回加载数量。"""
        with self._lock:
            count = self._wp_mgr.load_csv(csv_text)
            return count

    def start(self) -> bool:
        """开始自主导航。需要有航点且 GPS fix_quality >= 1。

        Returns:
            True 表示成功启动，False 表示条件不满足。
        """
        with self._lock:
            if self._state == NavState.NAVIGATING:
                logger.warning("NavigationEngine: 已在导航中，忽略重复 start()")
                return False
            if self._wp_mgr.current is None:
                logger.warning("NavigationEngine: 无有效航点，无法启动")
                return False
            if self._fix_quality < 1:
                logger.warning(f"NavigationEngine: GPS fix_quality={self._fix_quality}，不足以导航")
                return False

            self._wp_mgr.reset()
            # 不重置滤波器——保留已有 GPS 历史以便立即可用
            # 重置控制器 PID 避免积分饱和残留
            self._p2p_ctrl.reset()
            self._pp_ctrl.reset()
            self._gps_warning_sent = False
            self._state = NavState.NAVIGATING
            self._last_control_ts = time.time()

        logger.info(
            f"NavigationEngine: 导航开始，模式={self._nav_mode.value}，"
            f"滤波={self._filter_mode.value}，航点数={self._wp_mgr.progress[1]}"
        )
        self._schedule_broadcast()
        return True

    def stop(self) -> None:
        """停止导航，发送停车指令。"""
        with self._lock:
            if self._state == NavState.IDLE:
                return
            self._state = NavState.IDLE
            self._p2p_ctrl.reset()
            self._pp_ctrl.reset()

        self._send_velocity(0.0, 0.0)
        logger.info("NavigationEngine: 导航停止")
        self._schedule_broadcast()

    def set_nav_mode(self, mode: NavMode) -> None:
        with self._lock:
            self._nav_mode = mode
            self._p2p_ctrl.reset()
            self._pp_ctrl.reset()
        logger.info(f"NavigationEngine: 导航模式切换 → {mode.value}")

    def set_filter_mode(self, mode: FilterMode) -> None:
        with self._lock:
            self._filter_mode = mode
        logger.info(f"NavigationEngine: 滤波器切换 → {mode.value}")

    def get_status(self) -> dict:
        """返回当前导航状态字典（线程安全）。"""
        with self._lock:
            wp = self._wp_mgr.current
            prog = self._wp_mgr.progress
            distance_m = None
            target_bearing = None
            tolerance_m    = None

            if wp is not None and self._filter_mode == FilterMode.MOVING_AVG:
                pos = self._ma_filter.get_position()
                if pos != (0.0, 0.0):
                    distance_m     = haversine_distance(pos[0], pos[1], wp.lat, wp.lon)
                    target_bearing = bearing_to_target(pos[0], pos[1], wp.lat, wp.lon)
                    tolerance_m    = wp.tolerance_m
            elif wp is not None:
                pos = self._kf_filter.get_position()
                if pos != (0.0, 0.0):
                    distance_m     = haversine_distance(pos[0], pos[1], wp.lat, wp.lon)
                    target_bearing = bearing_to_target(pos[0], pos[1], wp.lat, wp.lon)
                    tolerance_m    = wp.tolerance_m

            return {
                "type":           "nav_status",
                "state":          self._state.name.lower(),
                "progress":       list(prog),
                "distance_m":     round(distance_m, 2) if distance_m is not None else None,
                "target_bearing": round(target_bearing, 1) if target_bearing is not None else None,
                "nav_mode":       self._nav_mode.value,
                "filter_mode":    self._filter_mode.value,
                "tolerance_m":    tolerance_m,
                "fix_quality":    self._fix_quality,
            }

    # ── 传感器回调 ────────────────────────────────────────
    def on_imu(self, imu_data: dict) -> None:
        """IMU 数据回调（20 Hz）。

        更新机器人朝向，驱动卡尔曼预测步骤，调用控制步骤。
        """
        try:
            compass = imu_data.get("compass", {})
            calibrated = compass.get("calibrated", False)
            if not calibrated:
                return

            bearing = compass.get("bearing", 0.0)
            now = time.time()
            dt = now - self._last_imu_ts if self._last_imu_ts > 0 else 0.05
            self._last_imu_ts = now

            with self._lock:
                self._robot_bearing = bearing

                # 卡尔曼预测（用 IMU 加速度作为控制输入）
                if self._filter_mode == FilterMode.KALMAN and self._kf_filter.is_ready:
                    accel = imu_data.get("accel", {})
                    ax, ay = accel.get("x", 0.0), accel.get("y", 0.0)
                    # 机体加速度（前向=x，侧向=y）旋转到 NED
                    rad = math.radians(bearing)
                    a_north = ax * math.cos(rad) - ay * math.sin(rad)
                    a_east  = ax * math.sin(rad) + ay * math.cos(rad)
                    self._kf_filter.predict(dt, a_north, a_east)

                if self._state != NavState.NAVIGATING:
                    return

            # 控制步骤（持锁外执行以减少锁持有时长）
            self._control_step(now)

        except Exception as e:
            logger.error(f"NavigationEngine.on_imu: {e}")

    def on_rtk(self, rtk_data: dict) -> None:
        """RTK GPS 数据回调（1 Hz）。

        推送 GPS 到活跃滤波器，检测 GPS 超时。
        """
        try:
            lat = rtk_data.get("lat")
            lon = rtk_data.get("lon")
            fq  = rtk_data.get("fix_quality", 0)

            with self._lock:
                self._fix_quality = fq

            if lat is None or lon is None or fq < 1:
                return

            with self._lock:
                self._last_gps_ts  = time.time()
                self._gps_warning_sent = False

                if self._filter_mode == FilterMode.MOVING_AVG:
                    self._ma_filter.update(lat, lon)
                else:
                    self._kf_filter.update(lat, lon, fq)

        except Exception as e:
            logger.error(f"NavigationEngine.on_rtk: {e}")

    # ── 控制步骤（内部，每 50ms）──────────────────────────
    def _control_step(self, now: float) -> None:
        """核心控制循环，由 on_imu 以约 20 Hz 调用。"""
        with self._lock:
            if self._state != NavState.NAVIGATING:
                return

            # GPS 超时检测
            if self._last_gps_ts > 0:
                gps_age = now - self._last_gps_ts
                if gps_age > NAV_GPS_TIMEOUT_S and not self._gps_warning_sent:
                    logger.warning(f"NavigationEngine: GPS 超时 {gps_age:.1f}s，暂停导航")
                    self._gps_warning_sent = True
                    self._send_velocity(0.0, 0.0)
                    self._schedule_broadcast_unsafe({"type": "nav_warning", "msg": "GPS timeout"})
                    return
                if self._gps_warning_sent:
                    return  # 超时中，等 GPS 恢复（on_rtk 会清除 _gps_warning_sent）

            # 获取滤波后的位置
            if self._filter_mode == FilterMode.MOVING_AVG:
                if not self._ma_filter.is_ready:
                    return
                pos = self._ma_filter.get_position()
            else:
                if not self._kf_filter.is_ready:
                    return
                pos = self._kf_filter.get_position()

            robot_lat, robot_lon = pos

            if self._robot_bearing is None:
                return  # 罗盘未校准

            wp = self._wp_mgr.current
            if wp is None:
                return

            dt = now - self._last_control_ts
            self._last_control_ts = now
            if dt <= 0 or dt > 1.0:
                dt = 0.05

            # 距离和到达判定
            distance = haversine_distance(robot_lat, robot_lon, wp.lat, wp.lon)
            switched = self._wp_mgr.update(distance, self._fix_quality)

            if switched:
                if self._wp_mgr.is_finished:
                    # 全部航点完成
                    self._state = NavState.FINISHED
                    total = self._wp_mgr.progress[1]
                    logger.info(f"NavigationEngine: 所有 {total} 个航点完成！")
                    self._send_velocity(0.0, 0.0)
                    self._schedule_broadcast_unsafe({"type": "nav_complete", "total_wp": total})
                    self._schedule_broadcast_unsafe(self._get_status_unsafe())
                    return
                else:
                    # 切换到下一个航点
                    self._p2p_ctrl.reset()
                    self._pp_ctrl.reset()
                    wp = self._wp_mgr.current

            # 计算控制输出
            bearing = self._robot_bearing
            if self._nav_mode == NavMode.PURE_PURSUIT:
                linear, angular = self._pp_ctrl.compute(
                    robot_lat, robot_lon, bearing,
                    self._wp_mgr.waypoints, self._wp_mgr.current_index, dt,
                )
            else:
                linear, angular = self._p2p_ctrl.compute(
                    robot_lat, robot_lon, bearing, wp, dt,
                )

            # 限幅
            linear  = max(-MAX_LINEAR_VEL,  min(MAX_LINEAR_VEL,  linear))
            angular = max(-MAX_ANGULAR_VEL, min(MAX_ANGULAR_VEL, angular))

        self._send_velocity(linear, angular)

        # 4Hz 广播状态
        self._broadcast_counter += 1
        if self._broadcast_counter % 5 == 0:
            self._schedule_broadcast()

    # ── 广播辅助 ─────────────────────────────────────────
    def _schedule_broadcast(self) -> None:
        """从工作线程安全地调度异步广播（不持锁）。"""
        status = self.get_status()
        asyncio.run_coroutine_threadsafe(self._broadcast(status), self._loop)

    def _schedule_broadcast_unsafe(self, msg: dict) -> None:
        """已持锁时调度广播（从锁内调用，不再获取锁）。"""
        asyncio.run_coroutine_threadsafe(self._broadcast(msg), self._loop)

    def _get_status_unsafe(self) -> dict:
        """不加锁的 get_status 内部版本（已持锁时调用）。"""
        wp = self._wp_mgr.current
        prog = self._wp_mgr.progress
        return {
            "type":           "nav_status",
            "state":          self._state.name.lower(),
            "progress":       list(prog),
            "distance_m":     None,
            "target_bearing": None,
            "nav_mode":       self._nav_mode.value,
            "filter_mode":    self._filter_mode.value,
            "tolerance_m":    wp.tolerance_m if wp else None,
            "fix_quality":    self._fix_quality,
        }
