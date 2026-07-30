"""
Plain ctypes wrapper for PTV's DrivingSimulatorProxy.dll.

Mirrors the surface declared in
`<VISSIM install>/API/DrivingSimulator_DLL/include/DrivingSimulatorProxy.h`.

No FIXS code dependencies on purpose. This is a Stage 1 probe (issue #156)
to characterize the DLL contract before any TrafficLayer integration.
"""

from __future__ import annotations

import ctypes
import os
from ctypes import (
    POINTER,
    c_bool,
    c_char,
    c_double,
    c_int,
    c_ushort,
    c_wchar_p,
)
from pathlib import Path

NAME_MAX_LENGTH = 100
MAX_UDA = 16


# enums - mirror DrivingSimulatorProxy.h
class TurningIndicator:
    LEFT = 1
    NONE = 0
    RIGHT = -1


class SignalState:
    RED = 1
    RED_AMBER = 2
    GREEN = 3
    AMBER = 4
    OFF = 5
    UNDEFINED = 6
    FLASHING_AMBER = 7
    FLASHING_RED = 8
    FLASHING_GREEN = 9
    ALTERNATING_RED_GREEN = 10
    GREEN_AMBER = 11

    _names = {
        1: "Red", 2: "RedAmber", 3: "Green", 4: "Amber", 5: "Off",
        6: "Undefined", 7: "FlashAmber", 8: "FlashRed", 9: "FlashGreen",
        10: "AltRG", 11: "GreenAmber",
    }

    @classmethod
    def name(cls, value: int) -> str:
        return cls._names.get(value, f"?({value})")


class Simulator_Veh_Data(ctypes.Structure):
    _fields_ = [
        ("VehicleID", c_int),
        ("VehicleType", c_int),
        ("Position_X", c_double),
        ("Position_Y", c_double),
        ("Position_Z", c_double),
        ("Orient_Heading", c_double),
        ("Orient_Pitch", c_double),
        ("Speed", c_double),
        ("Create", c_bool),
        ("CreateID", c_int),
        ("Delete", c_bool),
        ("ControlledByVissim", c_bool),
        ("RoutingDecisionNo", c_int),
        ("RouteNo", c_int),
    ]


class Simulator_Ped_Data(ctypes.Structure):
    _fields_ = [
        ("Position_X", c_double),
        ("Position_Y", c_double),
        ("Position_Z", c_double),
        ("Orient_Heading", c_double),
        ("DistanceSinceBirth", c_double),
        ("Speed", c_double),
    ]


class VISSIM_Veh_Data(ctypes.Structure):
    _fields_ = [
        ("VehicleID", c_int),
        ("VehicleType", c_int),
        ("ModelFileName", c_char * NAME_MAX_LENGTH),
        ("color", c_int),
        ("Position_X", c_double),
        ("Position_Y", c_double),
        ("Position_Z", c_double),
        ("Orient_Heading", c_double),
        ("Orient_Pitch", c_double),
        ("Speed", c_double),
        ("LeadingVehicleID", c_int),
        ("TrailingVehicleID", c_int),
        ("LinkID", c_int),
        ("LinkName", c_char * NAME_MAX_LENGTH),
        ("LinkCoordinate", c_double),
        ("LaneIndex", c_int),
        ("TurningIndicator", c_int),
        ("PreviousIndex", c_int),
        ("NumUDAs", c_int),
        ("UDA", c_double * MAX_UDA),
        ("CreateID", c_int),
        ("ControlledByVissim", c_bool),
    ]


class VISSIM_Sig_Data(ctypes.Structure):
    _fields_ = [
        ("ControllerID", c_int),
        ("SignalGroupID", c_int),
        ("SignalState", c_int),
    ]


class DSProxy:
    """ctypes wrapper around DrivingSimulatorProxy.dll."""

    def __init__(self, dll_path: str | os.PathLike):
        dll_path = str(dll_path)
        if not Path(dll_path).is_file():
            raise FileNotFoundError(f"DSProxy DLL not found: {dll_path}")
        self._dll_path = dll_path
        self._dll = ctypes.CDLL(dll_path)
        self._bind()

    def _bind(self) -> None:
        d = self._dll

        d.VISSIM_Connect.argtypes = [
            c_ushort,        # versionNo
            c_wchar_p,       # networkFileName
            c_ushort,        # simulatorFrequency
            c_double,        # visibilityRadius
            c_ushort,        # maxSimulatorVeh
            c_ushort,        # maxSimulatorPed
            c_ushort,        # maxSimulatorDet
            c_ushort,        # maxTotalVeh
            c_ushort,        # maxVissimPed
            c_ushort,        # maxVissimSigGrp
        ]
        d.VISSIM_Connect.restype = c_bool

        d.VISSIM_Disconnect.argtypes = []
        d.VISSIM_Disconnect.restype = c_bool

        d.VISSIM_SetDriverVehicles.argtypes = [c_int, POINTER(Simulator_Veh_Data)]
        d.VISSIM_SetDriverVehicles.restype = c_bool

        d.VISSIM_SetDriverPedestrians.argtypes = [c_int, POINTER(Simulator_Ped_Data)]
        d.VISSIM_SetDriverPedestrians.restype = c_bool

        d.VISSIM_SetDriverVehiclesAndPedestrians.argtypes = [
            c_int, POINTER(Simulator_Veh_Data),
            c_int, POINTER(Simulator_Ped_Data),
        ]
        d.VISSIM_SetDriverVehiclesAndPedestrians.restype = c_bool

        d.VISSIM_SetDetection.argtypes = [c_int, c_int]
        d.VISSIM_SetDetection.restype = c_bool

        d.VISSIM_DataReady.argtypes = []
        d.VISSIM_DataReady.restype = c_bool

        d.VISSIM_GetTrafficVehicles.argtypes = [
            POINTER(c_int), POINTER(POINTER(VISSIM_Veh_Data))
        ]
        d.VISSIM_GetTrafficVehicles.restype = None

        d.VISSIM_GetTrafficPedestrians.argtypes = [
            POINTER(c_int), POINTER(POINTER(ctypes.c_void_p))
        ]
        d.VISSIM_GetTrafficPedestrians.restype = None

        d.VISSIM_GetVehicleLists.argtypes = [
            POINTER(c_int), POINTER(POINTER(c_int)), POINTER(POINTER(c_int)),
            POINTER(c_int), POINTER(POINTER(c_int)),
            POINTER(c_int), POINTER(POINTER(c_int)),
        ]
        d.VISSIM_GetVehicleLists.restype = None

        d.VISSIM_GetSignalStates.argtypes = [
            POINTER(c_int), POINTER(POINTER(VISSIM_Sig_Data))
        ]
        d.VISSIM_GetSignalStates.restype = None

        d.VISSIM_GetLastErrorMessage.argtypes = []
        d.VISSIM_GetLastErrorMessage.restype = c_wchar_p

    def connect(
        self,
        version_no: int,
        network_file: str | os.PathLike,
        *,
        simulator_frequency: int = 10,
        visibility_radius: float = -1.0,
        max_simulator_veh: int = 10,
        max_simulator_ped: int = 10,
        max_simulator_det: int = 0,
        max_total_veh: int = 50000,
        max_vissim_ped: int = 50000,
        max_vissim_sig_grp: int = 1000,
    ) -> bool:
        net = str(Path(network_file).resolve())
        return bool(self._dll.VISSIM_Connect(
            version_no, net,
            simulator_frequency, visibility_radius,
            max_simulator_veh, max_simulator_ped, max_simulator_det,
            max_total_veh, max_vissim_ped, max_vissim_sig_grp,
        ))

    def disconnect(self) -> bool:
        return bool(self._dll.VISSIM_Disconnect())

    def last_error(self) -> str:
        msg = self._dll.VISSIM_GetLastErrorMessage()
        return msg if msg else ""

    def set_driver_vehicles(self, vehicles: list[Simulator_Veh_Data]) -> bool:
        n = len(vehicles)
        if n == 0:
            return bool(self._dll.VISSIM_SetDriverVehicles(0, None))
        arr = (Simulator_Veh_Data * n)(*vehicles)
        return bool(self._dll.VISSIM_SetDriverVehicles(n, arr))

    def data_ready(self) -> bool:
        return bool(self._dll.VISSIM_DataReady())

    def get_traffic_vehicles(self) -> list[VISSIM_Veh_Data]:
        n = c_int(0)
        ptr = POINTER(VISSIM_Veh_Data)()
        self._dll.VISSIM_GetTrafficVehicles(ctypes.byref(n), ctypes.byref(ptr))
        if n.value <= 0 or not ptr:
            return []
        # Copy into Python list so we don't hold a pointer into VISSIM's buffer
        return [VISSIM_Veh_Data.from_buffer_copy(ptr[i]) for i in range(n.value)]

    def get_vehicle_lists(self) -> dict:
        num_new = c_int(0)
        new_ids = POINTER(c_int)()
        new_types = POINTER(c_int)()
        num_moved = c_int(0)
        moved_ids = POINTER(c_int)()
        num_deleted = c_int(0)
        deleted_ids = POINTER(c_int)()
        self._dll.VISSIM_GetVehicleLists(
            ctypes.byref(num_new), ctypes.byref(new_ids), ctypes.byref(new_types),
            ctypes.byref(num_moved), ctypes.byref(moved_ids),
            ctypes.byref(num_deleted), ctypes.byref(deleted_ids),
        )
        return {
            "new":     [(new_ids[i], new_types[i]) for i in range(num_new.value)],
            "moved":   [moved_ids[i]   for i in range(num_moved.value)],
            "deleted": [deleted_ids[i] for i in range(num_deleted.value)],
        }

    def get_signal_states(self) -> list[VISSIM_Sig_Data]:
        n = c_int(0)
        ptr = POINTER(VISSIM_Sig_Data)()
        self._dll.VISSIM_GetSignalStates(ctypes.byref(n), ctypes.byref(ptr))
        if n.value <= 0 or not ptr:
            return []
        return [VISSIM_Sig_Data.from_buffer_copy(ptr[i]) for i in range(n.value)]
