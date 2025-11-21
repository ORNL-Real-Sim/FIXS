#******************************************************************************
#**  CarMaker - Version 13.1.3
#**  Vehicle Dynamics Simulation Toolkit
#**
#**  Copyright (C)   IPG Automotive GmbH
#**                  Germany                      WWW    www.ipg-automotive.com
#******************************************************************************
#**
#**  Build Configuration Script for ConfigurationDesk
#**
#******************************************************************************

from win32com.client import Dispatch, DispatchWithEvents

import collections
import os
import io
import re
import sys
import subprocess
import string
import time
import wx

from collections import OrderedDict

SOURCETYPE_SIMULINK_MODEL = 3



def ShowMsg(text):
    from win32con import MB_TOPMOST, MB_OK
    from win32ui import MessageBox

    MessageBox(text, CM_BUILD_CFG_NAME, MB_TOPMOST | MB_OK)


def CStr(obj):
    if obj is None:
        return ""
    else:
        return str(obj)


def SplitString(strValue, semicolonSeparated = False, stripQuotes = False):
    res = []
    if len(strValue.strip()) > 0:
        if semicolonSeparated:
            res = strValue.split(";")
        else:
            res = strValue.split()
        for i in range(len(res)):
            res[i] = res[i].strip()
            if stripQuotes:
                res[i] = res[i].strip("\"'")
    return res


def JoinStrings(strVec, separator = ", "):
    if strVec is None:
        return ""
    return separator.join(strVec)


def JoinPaths(strVec, separator = ", "):
    paths = JoinStrings(strVec, separator)
    return paths.replace("/", "\\")


def NormPath(path):
    return path.replace("/", "\\")


# Target architecture depends on dSPACE Release:
# * Up to 2020-A:        QNX (dsrt)
# * 2020-B up to 2021-B: Linux 32-Bit (dsrtlx)
# * 2022-A up to 2023-A: Linux 64-Bit (dsrt64), optionally 32-Bit (dsrtlx)
# * 2023-B and newer:    Linux 64-Bit (dsrt64)
def GetCM_Arch(cmdir):
    # Uncomment the following line and enter the desired CarMaker target platform, if the target platform does not fit
    # ConfigurationDesk defaults (e.g. QNX compiler with versions newer than "2020-A"):
    #return "dsrt"
    try:
        cmpu  = NormPath("%s/bin/cmplugutil.exe" %(cmdir))
        dsver = subprocess.check_output([cmpu, "getDSVersion"], universal_newlines=True)
        vsstr = dsver.strip()
    except:
        vsstr = ""
    if vsstr >= "2023-B":
        return "dsrt64"
    try:
        bmgmnt = Application.ActiveApplication.BuildManagement
        archid = bmgmnt.Properties.Item('TargetArchitecture').Value
    except:
        archid = 0
    if archid == 1:
        return "dsrtlx"
    elif archid == 2:
        return "dsrt64"
    if vsstr >= "2022-A":
        return "dsrt64"
    elif vsstr >= "2020-B":
        return "dsrtlx"
    return "dsrt"


CM_BUILD_CFG_NAME   = "CarMaker Build Configuration"
CM_BUILD_CFG_SCRIPT = "CM_BuildConfig.py"
CM4SL_BLOCK_NAME    = "CarMaker"
CM_GUI_EXE          = "CM_HIL.exe"
CM_SRC_DIR          = "src_cm4sl"
RMA_BUILD_CFG_NAME  = "RMA Slave Model Build Configuration"

CARMAKER_DIR    = "C:/IPG/carmaker/win64-13.1.3"
CARMAKER_VER    = "13.1.3"
CARMAKER_NUMVER = 130103

ARCH  = GetCM_Arch(CARMAKER_DIR)
ARCHF = "win32"

CARMAKER_BIN_DIR = "%s/bin"     %(CARMAKER_DIR)
CARMAKER_LIB_DIR = "%s/lib-%s"  %(CARMAKER_DIR, ARCH)
CARMAKER_INC_DIR = "%s/include" %(CARMAKER_DIR)
CARMAKER_GUI_DIR = "%s/GUI"     %(CARMAKER_DIR)
CARMAKER_PY_DIR  = "%s/Python"  %(CARMAKER_DIR)

CARMAKER_LIB = "%s/libcarmaker4sl.a" %(CARMAKER_LIB_DIR)
CAR_LIB      = "%s/libcar4sl.a"      %(CARMAKER_LIB_DIR)
MCYCLE_LIB   = "%s/libmcycle4sl.a"   %(CARMAKER_LIB_DIR)
TRUCK_LIB    = "%s/libtruck4sl.a"    %(CARMAKER_LIB_DIR)

DRIVER_LIB     = "%s/libipgdriver.a"    %(CARMAKER_LIB_DIR)
ROAD_LIB       = "%s/libipgroad.a"      %(CARMAKER_LIB_DIR)
TIRE_LIB       = "%s/libipgtire.a"      %(CARMAKER_LIB_DIR)
APO_LIB        = "%s/libapo.a"          %(CARMAKER_LIB_DIR)
INFO_LIB       = "%s/libinfofile.a"     %(CARMAKER_LIB_DIR)
Z_LIB          = "%s/libz-%s.a"         %(CARMAKER_LIB_DIR, ARCH)
URI_LIB        = "%s/liburiparser-%s.a" %(CARMAKER_LIB_DIR, ARCH)
USBLOADER_LIB  = "%s/libusbloader.a"    %(CARMAKER_LIB_DIR)
RMA_LIB        = "%s/librma.a"          %(CARMAKER_LIB_DIR)

CM_CFLAGS   = ("-include", "ipgrt.h")
RMA_CFLAGS  = ()
if ARCH == 'dsrtlx':
    CM_DEFINES = ["RS_CAVE", "RS_DEBUG", "RS_DSPACE", "DSPACE", "DSRTLX", "_DSRTLX"]
elif ARCH == 'dsrt64':
    CM_DEFINES = ["RS_CAVE", "RS_DEBUG", "RS_DSPACE", "DSPACE", "DSRT64", "_DSRT64"]
else:
    CM_DEFINES = ["RS_CAVE", "RS_DEBUG", "RS_DSPACE", "DSPACE", "DSRT", "_DSRT"]
CM_DEFINES.extend(("CM_HIL", "CM_NUMVER=%d" %(CARMAKER_NUMVER), "CM4SLDS"))
RMA_DEFINES = CM_DEFINES[:]
RMA_DEFINES.append("RMA_CLIENT")
if ARCH == 'dsrt':
    CM_DEFINES.append("USE_IPGRT_FUNCS")

SRC_DIRS  = (CM_SRC_DIR, "include")
SRC_FILES = ("CM_Main.c", "CM_Vehicle.c", "User.c", "IO.c", "app_tmp.c")

PROJECT_DIR = ""

PROP_COMPILER_DEFINES                  = "CompilerDefines"
PROP_COMPILER_UNDEFINES                = "CompilerUndefines"
PROP_C_COMPILER_OPTIONS                = "CCompilerOptions"
PROP_CPP_COMPILER_OPTIONS              = "CppCompilerOptions"
PROP_COMPILER_OPTIMIZATION_SET         = "CompilerOptimizationSet"
PROP_USER_DEFINED_OPTIMIZATION_OPTIONS = "UserDefinedOptimizationOptions"
PROP_SEARCH_PATHS                      = "SearchPaths"
PROP_CUSTOM_SOURCE_FILES               = "CustomSourceFiles"
PROP_CUSTOM_LIBRARIES                  = "CustomLibraries"

RelevantPropertyNames = (PROP_COMPILER_DEFINES,
                         PROP_COMPILER_UNDEFINES,
                         PROP_C_COMPILER_OPTIONS,
                         PROP_CPP_COMPILER_OPTIONS,
                         PROP_COMPILER_OPTIMIZATION_SET,
                         PROP_USER_DEFINED_OPTIMIZATION_OPTIONS,
                         PROP_SEARCH_PATHS,
                         PROP_CUSTOM_SOURCE_FILES,
                         PROP_CUSTOM_LIBRARIES)

try:
    pyvdir = ("python%d.%d" %(sys.version_info.major, sys.version_info.minor))
    sys.path.append(NormPath(CARMAKER_PY_DIR + "/" + pyvdir))
    import infofiles
    Have_InfoFiles = True
except:
    Have_InfoFiles = False


# =============================================================================
# DDE Client
# =============================================================================
from ctypes import POINTER, WINFUNCTYPE, c_char_p, c_void_p, c_int, c_ulong
from ctypes.wintypes import BOOL, DWORD, BYTE, INT, LPCWSTR, UINT, ULONG

# DECLARE_HANDLE(name) typedef void *name;
HCONV     = c_void_p  # = DECLARE_HANDLE(HCONV)
HDDEDATA  = c_void_p  # = DECLARE_HANDLE(HDDEDATA)
HSZ       = c_void_p  # = DECLARE_HANDLE(HSZ)
LPBYTE    = c_char_p  # POINTER(BYTE)
LPDWORD   = POINTER(DWORD)
LPSTR     = c_char_p
ULONG_PTR = c_ulong

PM_NOREMOVE = 0x0000
PM_REMOVE   = 0x0001
PM_NOYIELD  = 0x0002

#PM_QS_INPUT       = (QS_INPUT << 16)
#PM_QS_PAINT       = (QS_PAINT << 16)
#PM_QS_POSTMESSAGE = ((QS_POSTMESSAGE | QS_HOTKEY | QS_TIMER) << 16)
#PM_QS_SENDMESSAGE = (QS_SENDMESSAGE << 16)


# See windows/ddeml.h for declaration of struct CONVCONTEXT
PCONVCONTEXT = c_void_p

DMLERR_NO_ERROR = 0

# Predefined Clipboard Formats
CF_TEXT         =  1
CF_BITMAP       =  2
CF_METAFILEPICT =  3
CF_SYLK         =  4
CF_DIF          =  5
CF_TIFF         =  6
CF_OEMTEXT      =  7
CF_DIB          =  8
CF_PALETTE      =  9
CF_PENDATA      = 10
CF_RIFF         = 11
CF_WAVE         = 12
CF_UNICODETEXT  = 13
CF_ENHMETAFILE  = 14
CF_HDROP        = 15
CF_LOCALE       = 16
CF_DIBV5        = 17
CF_MAX          = 18

DDE_FACK          = 0x8000
DDE_FBUSY         = 0x4000
DDE_FDEFERUPD     = 0x4000
DDE_FACKREQ       = 0x8000
DDE_FRELEASE      = 0x2000
DDE_FREQUESTED    = 0x1000
DDE_FAPPSTATUS    = 0x00FF
DDE_FNOTPROCESSED = 0x0000

DDE_FACKRESERVED  = (~(DDE_FACK | DDE_FBUSY | DDE_FAPPSTATUS))
DDE_FADVRESERVED  = (~(DDE_FACKREQ | DDE_FDEFERUPD))
DDE_FDATRESERVED  = (~(DDE_FACKREQ | DDE_FRELEASE | DDE_FREQUESTED))
DDE_FPOKRESERVED  = (~(DDE_FRELEASE))

XTYPF_NOBLOCK        = 0x0002
XTYPF_NODATA         = 0x0004
XTYPF_ACKREQ         = 0x0008

XCLASS_MASK          = 0xFC00
XCLASS_BOOL          = 0x1000
XCLASS_DATA          = 0x2000
XCLASS_FLAGS         = 0x4000
XCLASS_NOTIFICATION  = 0x8000

XTYP_ERROR           = (0x0000 | XCLASS_NOTIFICATION | XTYPF_NOBLOCK)
XTYP_ADVDATA         = (0x0010 | XCLASS_FLAGS)
XTYP_ADVREQ          = (0x0020 | XCLASS_DATA | XTYPF_NOBLOCK)
XTYP_ADVSTART        = (0x0030 | XCLASS_BOOL)
XTYP_ADVSTOP         = (0x0040 | XCLASS_NOTIFICATION)
XTYP_EXECUTE         = (0x0050 | XCLASS_FLAGS)
XTYP_CONNECT         = (0x0060 | XCLASS_BOOL | XTYPF_NOBLOCK)
XTYP_CONNECT_CONFIRM = (0x0070 | XCLASS_NOTIFICATION | XTYPF_NOBLOCK)
XTYP_XACT_COMPLETE   = (0x0080 | XCLASS_NOTIFICATION )
XTYP_POKE            = (0x0090 | XCLASS_FLAGS)
XTYP_REGISTER        = (0x00A0 | XCLASS_NOTIFICATION | XTYPF_NOBLOCK )
XTYP_REQUEST         = (0x00B0 | XCLASS_DATA )
XTYP_DISCONNECT      = (0x00C0 | XCLASS_NOTIFICATION | XTYPF_NOBLOCK )
XTYP_UNREGISTER      = (0x00D0 | XCLASS_NOTIFICATION | XTYPF_NOBLOCK )
XTYP_WILDCONNECT     = (0x00E0 | XCLASS_DATA | XTYPF_NOBLOCK)
XTYP_MONITOR         = (0x00F0 | XCLASS_NOTIFICATION | XTYPF_NOBLOCK)

XTYP_MASK            = 0x00F0
XTYP_SHIFT           = 4

TIMEOUT_ASYNC        = 0xFFFFFFFF

def get_winfunc(libname, funcname, restype=None, argtypes=(), _libcache={}):
    """Retrieve a function from a library, and set the data types."""
    from ctypes import windll

    if libname not in _libcache:
        _libcache[libname] = windll.LoadLibrary(libname)
    func = getattr(_libcache[libname], funcname)
    func.argtypes = argtypes
    func.restype = restype

    return func


DDECALLBACK = WINFUNCTYPE(HDDEDATA, UINT, UINT, HCONV, HSZ, HSZ, HDDEDATA,
                          ULONG_PTR, ULONG_PTR)

# =============================================================================
# Event Handling
# =============================================================================
from win32com import client

PrjCtrl = None

AppEvH = None


# =============================================================================
# Class: DDE
# =============================================================================
class DDE:
    """Object containing all the DDE functions"""
    AccessData         = get_winfunc("user32", "DdeAccessData",          LPBYTE,   (HDDEDATA, LPDWORD))
    ClientTransaction  = get_winfunc("user32", "DdeClientTransaction",   HDDEDATA, (LPBYTE, DWORD, HCONV, HSZ, UINT, UINT, DWORD, LPDWORD))
    Connect            = get_winfunc("user32", "DdeConnect",             HCONV,    (DWORD, HSZ, HSZ, PCONVCONTEXT))
    CreateStringHandle = get_winfunc("user32", "DdeCreateStringHandleW", HSZ,      (DWORD, LPCWSTR, UINT))
    Disconnect         = get_winfunc("user32", "DdeDisconnect",          BOOL,     (HCONV,))
    GetLastError       = get_winfunc("user32", "DdeGetLastError",        UINT,     (DWORD,))
    Initialize         = get_winfunc("user32", "DdeInitializeW",         UINT,     (LPDWORD, DDECALLBACK, DWORD, DWORD))
    FreeDataHandle     = get_winfunc("user32", "DdeFreeDataHandle",      BOOL,     (HDDEDATA,))
    FreeStringHandle   = get_winfunc("user32", "DdeFreeStringHandle",    BOOL,     (DWORD, HSZ))
    QueryString        = get_winfunc("user32", "DdeQueryStringA",        DWORD,    (DWORD, HSZ, LPSTR, DWORD, c_int))
    UnaccessData       = get_winfunc("user32", "DdeUnaccessData",        BOOL,     (HDDEDATA,))
    Uninitialize       = get_winfunc("user32", "DdeUninitialize",        BOOL,     (DWORD,))

# =============================================================================
# Class: DDEError
# =============================================================================
class DDEError(RuntimeError):
    """Exception raise when a DDE errpr occures."""
    def __init__(self, msg, idInst=None):
        if idInst is None:
            RuntimeError.__init__(self, msg)
        else:
            RuntimeError.__init__(self, "%s (err=%s)" %(msg, hex(DDE.GetLastError(idInst))))

# =============================================================================
# Class: DDEClient
# =============================================================================
class DDEClient:
    """The DDEClient class.

    Use this class to create and manage a connection to a service/topic.
    To get classbacks subclass DDEClient and overwrite callback."""

    def __init__(self, service, topic):
        """Create a connection to a service/topic."""
        from ctypes import byref

        self._idInst = DWORD(0)
        self._hConv = HCONV()

        self._callback = DDECALLBACK(self._callback)
        res = DDE.Initialize(byref(self._idInst), self._callback, 0x00000010, 0)
        if res != DMLERR_NO_ERROR:
            raise DDEError("Unable to register with DDEML (err=%s)" % hex(res))

        hszService = DDE.CreateStringHandle(self._idInst, service, 1200)
        hszTopic = DDE.CreateStringHandle(self._idInst, topic, 1200)
        self._hConv = DDE.Connect(self._idInst, hszService, hszTopic, PCONVCONTEXT())
        DDE.FreeStringHandle(self._idInst, hszTopic)
        DDE.FreeStringHandle(self._idInst, hszService)
        if not self._hConv:
            raise DDEError("Unable to establish a conversation with server", self._idInst)

    def __del__(self):
        """Cleanup any active connections."""
        if self._hConv:
            DDE.Disconnect(self._hConv)
        if self._idInst:
            DDE.Uninitialize(self._idInst)

    def advise(self, item, stop=False):
        """Request updates when DDE data changes."""
        from ctypes import byref

        hszItem = DDE.CreateStringHandle(self._idInst, item, 1200)
        hDdeData = DDE.ClientTransaction(LPBYTE(), 0, self._hConv, hszItem, CF_TEXT, XTYP_ADVSTOP if stop else XTYP_ADVSTART, TIMEOUT_ASYNC, LPDWORD())
        DDE.FreeStringHandle(self._idInst, hszItem)
        if not hDdeData:
            raise DDEError("Unable to %s advise" %("stop" if stop else "start"), self._idInst)
        DDE.FreeDataHandle(hDdeData)

    def execute(self, command, timeout=5000):
        """Execute a DDE command."""
        from ctypes import byref

        if sys.version < '3':
            pData = c_char_p(command).value
        else:
            pData = command.encode('utf-8')
        cbData = DWORD(len(pData) + 1)
        hDdeData = DDE.ClientTransaction(pData, cbData, self._hConv, HSZ(), CF_TEXT, XTYP_EXECUTE, timeout, LPDWORD())
        if not hDdeData:
            raise DDEError("Unable to send command", self._idInst)
        DDE.FreeDataHandle(hDdeData)

    def request(self, item, timeout=5000):
        """Request data from DDE service."""
        from ctypes import byref

        hszItem = DDE.CreateStringHandle(self._idInst, item, 1200)
        hDdeData = DDE.ClientTransaction(LPBYTE(), 0, self._hConv, hszItem, CF_TEXT, XTYP_REQUEST, timeout, LPDWORD())
        DDE.FreeStringHandle(self._idInst, hszItem)
        if not hDdeData:
            raise DDEError("Unable to request item", self._idInst)

        if timeout != TIMEOUT_ASYNC:
            pdwSize = DWORD(0)
            pData = DDE.AccessData(hDdeData, byref(pdwSize))
            if not pData:
                DDE.FreeDataHandle(hDdeData)
                raise DDEError("Unable to access data", self._idInst)
            # TODO: use pdwSize
            DDE.UnaccessData(hDdeData)
        else:
            pData = None
            DDE.FreeDataHandle(hDdeData)
        return pData

    def callback(self, value, item=None):
        """Callback function for advice."""
        print(("%s: %s" %(item, value)))

    def _callback(self, wType, uFmt, hConv, hsz1, hsz2, hDdeData, dwData1, dwData2):
        if wType == XTYP_ADVDATA:
            from ctypes import byref, create_string_buffer

            dwSize = DWORD(0)
            pData = DDE.AccessData(hDdeData, byref(dwSize))
            if pData:
                item = create_string_buffer('\000' * 128)
                DDE.QueryString(self._idInst, hsz2, item, 128, 1004)
                self.callback(pData, item.value)
                DDE.UnaccessData(hDdeData)
            return DDE_FACK
        return 0


# =============================================================================
# Class: CM_BuildEvents
# =============================================================================
class CM_BuildEvents:
    def OnStarted(self):
        pass

    def OnQuitting(self):
        pass

    def OnBuildStarting(self, x):
        EArgs = Dispatch(x)
        RMA_Topology_Update(self.ActiveApplication)

    def OnBuildStarted(self, x):
        EArgs = Dispatch(x)
        StartCM_GUI()

    def OnBuildFinished(self, x):
        EArgs = Dispatch(x)
        BRes = EArgs.Item("BuildResult")
        if BRes is None:
            return
        if BRes.Success:
            if Have_InfoFiles:
                RMA_ExtQuants_Create(self.ActiveApplication)
            else:
                RMA_ExtQuants_Create_Txt(self.ActiveApplication)
            if BRes.Downloaded:
                ConnectCM_GUI()


# =============================================================================
# Class: BuildConfigInfo
# =============================================================================
class BuildConfigInfo:
    def __init__(self, bcs=None, name=""):
        self.AppProcCount = 0
        self.CreateOnCfgDesk = False
        self.DeleteOnCfgDesk = False
        self.DisplayName = ""
        self.Properties = {}
        if bcs is None:
            self.Name = name
        else:
            self.Name = bcs.Name
            self.DisplayName = bcs.Name
            for p in bcs.Properties:
                if CStr(p.Value).count("COMObject") > 0:
                    elements = []
                    for e in p.Value:
                        elements.append(e)
                    self.Properties[p.Name] = ";".join(elements)
                else:
                    self.Properties[p.Name] = CStr(p.Value)

    def GetCompDefines(self):
        return CStr(self.Properties.get(PROP_COMPILER_DEFINES))

    def SetCompDefines(self, strValue):
        self.Properties[PROP_COMPILER_DEFINES] = strValue

    def GetCompUnDefines(self):
        return CStr(self.Properties.get(PROP_COMPILER_UNDEFINES))

    def SetCompUnDefines(self, strValue):
        self.Properties[PROP_COMPILER_UNDEFINES] = strValue

    def GetSearchPaths(self):
        return CStr(self.Properties.get(PROP_SEARCH_PATHS))

    def SetSearchPaths(self, strValue):
        self.Properties[PROP_SEARCH_PATHS] = strValue

    def GetCustomLibraries(self):
        return CStr(self.Properties.get(PROP_CUSTOM_LIBRARIES))

    def SetCustomLibraries(self, strValue):
        self.Properties[PROP_CUSTOM_LIBRARIES] = strValue

    def GetCustomSrcFiles(self):
        return CStr(self.Properties.get(PROP_CUSTOM_SOURCE_FILES))

    def SetCustomSrcFiles(self, strValue):
        self.Properties[PROP_CUSTOM_SOURCE_FILES] = strValue

    def GetCCompOpts(self):
        return CStr(self.Properties.get(PROP_C_COMPILER_OPTIONS))

    def SetCCompOpts(self, strValue):
        self.Properties[PROP_C_COMPILER_OPTIONS] = strValue

    def GetCPPCompOpts(self):
        return CStr(self.Properties.get(PROP_CPP_COMPILER_OPTIONS))

    def SetCPPCompOpts(self, strValue):
        self.Properties[PROP_CPP_COMPILER_OPTIONS] = strValue

    def IsEqual(self, bci):
        if self.Name != bci.Name:
            return False
        return self.PropertiesAreEqual(bci)

    def PropertiesAreEqual(self, bci):
        if len(self.Properties) != len(bci.Properties):
            return False
        for item in list(self.Properties.items()):
            if item[0] == "Name":
                continue
            prop = bci.Properties.get(item[0])
            if CStr(item[1]) != CStr(prop):
                return False
        return True

    def CloneProperties(self, obci):
        self.Properties.clear()
        for propItem in list(obci.Properties.items()):
            self.Properties[propItem[0]] = propItem[1]

    def UpdateWithBuildConfig(self, bcs):
        self.SetCompDefines(bcs.COMPILER_DEFINES)
        self.SetCompUnDefines(bcs.COMPILER_UNDEFINES)
        self.SetSearchPaths(bcs.SEARCH_PATHS)
        self.SetCustomLibraries(bcs.LIBRARIES)
        self.SetCustomSrcFiles(bcs.SOURCE_FILES)
        self.SetCCompOpts(bcs.COMPILER_OPTIONS)
        self.SetCPPCompOpts(bcs.COMPILER_OPTIONS)


# =============================================================================
# Class: ApplProcInfo
# =============================================================================
class ApplProcInfo:
    def __init__(self, name, bcsInfo):
        self.Name      = name
        self.BcsName   = bcsInfo.Name
        self.ModelName = ""
        self.Model     = None
        self.ModelPath = None


class BuildConfig(object):
    def __init__(self):
        # Initialize the ConfigurationDesk application object.
        self.CfgDesk = None
        # Initialize the application to perform a build process with.
        self.ActApp = None

        # Initialize Macro and Search Path template
        self.COMPILER_DEFINES   = ""
        self.COMPILER_UNDEFINES = ""
        self.COMPILER_OPTIONS   = ""
        self.COMPILER_OPT_SET   = ""
        self.INCLUDES           = ""
        self.USER_OPT_OPTIONS   = ""
        self.SOURCE_FILES       = ""
        self.LIBRARIES          = ""
        self.SEARCH_PATHS       = ""

        # Initialize Collections for Build Configuration Informations
        self.BuildCfgInfos = collections.OrderedDict()
        self.ApplProcInfos = collections.OrderedDict()

    def Initialize(self):
        try:
            # Get the ConfigurationDesk Automation interface
            self.CfgDesk = Dispatch("ConfigurationDesk.Application")
            self.CfgDesk.MainWindow.Visible = True
            if self.CfgDesk.ActiveApplication is None:
                # No active application, nothing to do
                return False
            self.ActApp = self.CfgDesk.ActiveApplication
            return True
        except Exception as exc:
            ShowMsg("Error initializing: " + str(exc))
            return False

    def GetApplProcInfosForBcs(self, bci):
        res = []
        for appi in list(self.ApplProcInfos.values()):
            if appi.BcsName == bci.Name:
                res.append(appi)
        return res

    def GetBuildConfigInfo(self, name, actApp=None):
        if actApp is None:
            actApp = self.ActApp
        if name in self.BuildCfgInfos:
            return self.BuildCfgInfos.get(name)
        return None

    def ReadBuildConfigs(self, actApp=None):
        # Read Build Configuration Sets
        if actApp is None:
            actApp = self.ActApp
        self.BuildCfgInfos.clear()
        self.ApplProcInfos.clear()
        if actApp is None:
            return False
        # get access to all BuildConfiguration relations of active app
        relBC = actApp.Relations.Item("BuildConfiguration")
        # get 1st executable application in active BuildConfiguration relations
        rootApp = relBC.GetTopNodes().Item(0)
        # collect ApplicationProcessOptions:
        # -> scan all relations of rootApp (executable application)
        for bcs in relBC.GetElements(rootApp):
            if not bcs.IsOfRole("ApplicationProcessOptions"):
                continue
            # read all Build Configuration Set informations
            bci = BuildConfigInfo(bcs=bcs)
            if bci.Name in self.BuildCfgInfos:
                self.BuildCfgInfos.clear()
                self.ApplProcInfos.clear()
                return False
            self.BuildCfgInfos[bci.Name] = bci
            # scan all relations of this Build Configuration Set
            for proc in relBC.GetElements(bcs):
                bci.AppProcCount += 1
                if relBC.GetElements(proc).Count <= 0:
                    continue
                model = relBC.GetElements(proc).Item(0)
                if model.Properties.Item("Source type").Value != SOURCETYPE_SIMULINK_MODEL:
                    continue
                appi = ApplProcInfo(proc.Name, bcs)
                appi.ModelName = model.Name
                appi.Model     = model
                for comp in actApp.Components.Item("ModelTopology"):
                    if comp.Name != appi.ModelName:
                        continue
                    prop = comp.Properties.TryGetItem("Model location")
                    if prop is None:
                        continue
                    appi.ModelPath = prop.Value
                if appi.Name in self.ApplProcInfos:
                    self.BuildCfgInfos.clear()
                    self.ApplProcInfos.clear()
                    return False
                self.ApplProcInfos[appi.Name] = appi
        return True

    def ApplyBuildConfig(self, bci=None):
        global PROJECT_DIR

        # get access to all BuildConfiguration relations of active app
        relBC = self.ActApp.Relations.Item("BuildConfiguration")
        # get 1st executable application in active BuildConfiguration relations
        rootApp = relBC.GetTopNodes().Item(0)
        bcsType = relBC.GetCreatableTypes(rootApp).Item("ApplicationProcessOptions")
        dictBCS = OrderedDict()
        dictAP  = OrderedDict()
        # Read the current configuration
        for bcs in relBC.GetElements(rootApp):
            if not bcs.IsOfRole("ApplicationProcessOptions"):
                continue
            dictBCS[bcs.Name] = bcs
            for ap in relBC.GetElements(bcs):
                dictAP[ap.Name] = ap

        bci_name = ""
        if bci is not None:
            bci_set = collections.OrderedDict()
            bci_set[bci.Name] = bci
            bci_name = bci.Name
        else:
            bci_set = self.BuildCfgInfos
        for bci in list(bci_set.values()):
            if bci.DeleteOnCfgDesk:
                # Delete the Build Configuration Info
                if bci.DisplayName in dictBCS:
                    relBC.RemoveElements(rootApp, [dictBCS[bci.DisplayName]])
                continue
            # Get the Application Processes to assign to the BCS
            appisToAssign = self.GetApplProcInfosForBcs(bci)
            # Get an existing BCS from CfgDesk or create a new one
            curBCS = None
            if bci.CreateOnCfgDesk:
                curBCS = relBC.CreateDataObject(bcsType, rootApp)
                curBCS.Name = bci.DisplayName
            else:
                if bci.DisplayName in dictBCS:
                    curBCS = dictBCS[bci.DisplayName]
            # If a BCS was found or created, assign the Application Processes
            if curBCS is None:
                continue
            for propName in RelevantPropertyNames:
                curBCS.Properties[propName].Value = CStr(bci.Properties.get(propName))
            for appi in appisToAssign:
                if appi.Name not in dictAP:
                    continue
                spaths = bci.Properties.get(PROP_SEARCH_PATHS)
                if curBCS.Name == bci_name and appi.Model is not None:
                    srch = [ CARMAKER_INC_DIR ]
                    if appi.ModelPath is not None:
                        mdir = os.path.dirname(appi.ModelPath)
                        pdir = os.path.dirname(mdir)
                        if pdir != "":
                            PROJECT_DIR = pdir
                            for d in SRC_DIRS:
                                srch.append("%s/%s" %(pdir, d))
                    spaths = JoinPaths(srch, "; ")
                curBCS.Properties[PROP_SEARCH_PATHS].Value = CStr(spaths)
                relBC.AddElements(curBCS, [dictAP[appi.Name]])
        return True


class CM_BuildConfig(BuildConfig):
    def Initialize(self):
        if not super(CM_BuildConfig, self).Initialize():
            return False
        self.COMPILER_DEFINES = JoinStrings(CM_DEFINES, " ")
        self.COMPILER_OPTIONS = JoinStrings(CM_CFLAGS, " ")
        srch = [ CARMAKER_INC_DIR ]
        self.SEARCH_PATHS = JoinPaths(srch, "; ")
        libs = [ "libdscandrv.so", "libRealSimDsLib_2024a_CM13_1_3.a" ]
        self.LIBRARIES = JoinPaths(libs, "; ")
        # srcf = [ "MySource_File.c" ]
        # self.SOURCE_FILES = JoinPaths(srcf, "; ")
        return True


class RMA_BuildConfig(BuildConfig):
    def Initialize(self):
        if not super(RMA_BuildConfig, self).Initialize():
            return False
        self.COMPILER_DEFINES = JoinStrings(RMA_DEFINES, " ")
        self.COMPILER_OPTIONS = JoinStrings(RMA_CFLAGS, " ")
        srch = [ CARMAKER_INC_DIR ]
        self.SEARCH_PATHS = JoinPaths(srch, "; ")
        # libs = [ "libmylib.so" ]
        # self.LIBRARIES = JoinPaths(libs, "; ")
        # srcf = [ "MySource_File.c" ]
        # self.SOURCE_FILES = JoinPaths(srcf, "; ")
        return True


# =============================================================================
# Class: ProjectController
# =============================================================================
class ProjectController(object):
    def __init__(self):
        # Initialize the ConfigurationDesk application object.
        self.CfgDesk = None

        # Initialize the project events object
        self.PrjEvH = None

    def Initialize(self):
        # Start ConfigurationDesk and set it to the demo controller class.
        self.CfgDesk = client.Dispatch("ConfigurationDesk.Application")

        # ConfigurationDesk starts windowless if started via automation.
        self.CfgDesk.MainWindow.Visible = True

    def ConnectToEvents(self):
        # The project events are accessible via the ICaProjectEvents interface.
        # The events provider is the ICaProjectManagement object, the
        # ProjectEvents class, which is defined below, must be used in the same
        # way like the ApplicationEvents.
        PrjMgmnt = self.CfgDesk.ProjectManagement
        self.PrjEvH = client.DispatchWithEvents(PrjMgmnt, ProjectEvents)

    def Delete(self):
        del self.PrjEvH
        del self.CfgDesk


# =============================================================================
# Class: ProjectEvents
# =============================================================================
class ProjectEvents(object):
    def OnProjectRootUpdating(self, args):
        pass

    def OnProjectRootUpdated(self, args):
        pass

    def OnProjectLoading(self, args):
        pass

    def OnProjectLoaded(self, args):
        pass

    def OnProjectClosing(self, args):
        DeleteProjectEventHandler()

    def OnProjectClosed(self, args):
        pass

    def OnProjectSaving(self, args):
        pass

    def OnProjectSaved(self, args):
        pass

    def OnApplicationLoading(self, args):
        pass

    def OnApplicationLoaded(self, args):
        MakeBuildConfig()

    def OnApplicationClosing(self, args):
        pass

    def OnApplicationClosed(self, args):
        pass

    def OnApplicationSaving(self, args):
        pass

    def OnApplicationSaved(self, args):
        pass


def CreateProjectEventHandler():
    global PrjCtrl

    if PrjCtrl is not None:
        DeleteProjectEventHandler()
    try:
        # Create the controller which helps to handle project settings.
        PrjCtrl = ProjectController()
        # Call the initialize function of the ProjectController.
        PrjCtrl.Initialize()
        # Connect to the events
        PrjCtrl.ConnectToEvents()
    except Exception as exc:
        ShowMsg("Exception in script CM_BuildConfig.py: " + str(exc))


def DeleteProjectEventHandler():
    global PrjCtrl

    if PrjCtrl is not None:
        try:
            PrjCtrl.Delete()
        except Exception as exc:
            ShowMsg("Exception in script CM_BuildConfig.py: " + str(exc))
        PrjCtrl = None


def UpdateBuildConfig(bctype):
    if bctype == "cm":
        bc = CM_BuildConfig()
        bcname = CM_BUILD_CFG_NAME
    elif bctype == "rma":
        bc = RMA_BuildConfig()
        bcname = RMA_BUILD_CFG_NAME
    else:
        return
    if bc is None or not bc.Initialize():
        return
    bc.ReadBuildConfigs()
    bci = bc.GetBuildConfigInfo(bcname)
    if bci is None:
        bci = BuildConfigInfo(name=bcname)
        bci.CreateOnCfgDesk = True
        bci.DisplayName     = bcname
    bci.UpdateWithBuildConfig(bc)
    if not bc.ApplyBuildConfig(bci):
        raise Exception(bcname)


def MakeBuildConfig():
    global AppEvH

    try:
        # Update CarMaker Build Configuration
        UpdateBuildConfig("cm")
        # Update RMA Build Configuration
        UpdateBuildConfig("rma")
    except Exception as exc:
        ShowMsg("Failed to apply Build Configuration settings (" + str(exc) + ")")
        return
    AppEvH = DispatchWithEvents("ConfigurationDesk.Application", CM_BuildEvents)
    return


def StartCM_GUI():
    global PROJECT_DIR
    TCL_DDE_EXECUTE_RESULT = "$TCLEVAL$EXECUTE$RESULT"
    cmdde = None
    try:
        cmdde = DDEClient("TclEval", "CarMaker")
        cmdde.execute("wm deiconify .; raise .")
    except:
        exepath = NormPath("%s/%s" %(CARMAKER_BIN_DIR, CM_GUI_EXE))
        subprocess.call(exepath)
        n_try = 0
        while n_try < 10:
            try:
                cmdde = DDEClient("TclEval", "CarMaker")
            except:
                time.sleep(1)
            n_try = n_try + 1
    finally:
        if not PROJECT_DIR or cmdde is None:
            return
        cmdde.execute("""
            set nprjdir [file normalize {%s}]
            if {$nprjdir ne $HIL(ProjDir)} {
                Project::Select $nprjdir
            }
            Appl::StartIPGRT""" %(PROJECT_DIR))
        del cmdde


def ConnectCM_GUI():
    TCL_DDE_EXECUTE_RESULT = "$TCLEVAL$EXECUTE$RESULT"
    cmdde = None
    try:
        cmdde = DDEClient("TclEval", "CarMaker")
        cmdde.execute("wm deiconify .; raise .")
        cmdde.execute("Appl::Connect [Appl::IPGRT_GetIP]", 10000)
    except:
        ShowMsg("CarMaker GUI: Connect failed!")
    finally:
        if cmdde is not None:
            del cmdde


def ModelContains(mdl, name):
    for i in range(mdl.GetCount()):
        try:
            sub = mdl.Item(i)
            if sub.Name == name:
                return True
        except:
            continue
        if ModelContains(sub, name):
            return True
    return False


def FindSubModel(mdl, name):
    for i in range(mdl.GetCount()):
        try:
            if mdl.Item(i).Name == name:
                return mdl.Item(i)
        except:
            continue
        sub = FindSubModel(mdl.Item(i), name)
        if sub is not None:
            return sub
    return None


def CreateRMA_Topology(dstdir, nd_tab, master_app):
    fname   = "rma_topology.h"
    dstname = os.path.join(dstdir, fname)
    tmpldir = os.path.join(CARMAKER_DIR, "Misc-CfgDesk")
    tmpl = os.path.join(tmpldir, fname + ".template")
    try:
        with io.open(tmpl, mode="rt", buffering=1, newline=None) as fin:
            with io.open(dstname, mode="wt", buffering=1, newline=None) as fout:
                ln = fin.readline()
                while ln != "":
                    if ln.strip() == "RMA_NODE_TABLE_NOT_DEFINED":
                        fout.write(nd_tab)
                    elif ln.strip() == "RMA_MASTER_APP_NOT_DEFINED":
                        fout.write("#define RMA_MasterApp\t\"" + master_app + "\"\n")
                    else:
                        fout.write(ln)
                    ln = fin.readline()
    except Exception as exc:
        ShowMsg("Error creating " + dstname + ": " + str(exc))
        return False
    return True


def RMA_Topology_Update(app):
    top = app.Components.Item("ModelTopology")
    mstrmdl = None
    nodetab = ""
    for i in range(top.Count):
        mdl = top.Item(i)
        if not mdl.IsInApplication:
            continue
        if ModelContains(mdl, CM4SL_BLOCK_NAME):
            if mstrmdl is not None:
                print("CarMaker can't be distributed on different models")
                return
            mstrmdl = mdl
        nodetab = nodetab + "    { \"" + mdl.Name + "\", " + str(i+1) + " },\n"
    if mstrmdl is None:
        return
    for mdl in top:
        if not mdl.IsInApplication:
            continue
        dstdir = os.path.dirname(mdl.Properties.Item("Model location").Value)
        CreateRMA_Topology(dstdir, nodetab, mstrmdl.Name)


def RMA_ExtQuants_Create(app):
    global PROJECT_DIR

    top = app.Components.Item("ModelTopology")
    exttab = []
    extqu = infofiles.IFile()
    extqu.setstr("FileIdent", "CarMaker-ExternalQuantities 1")
    extqu.settxt("Description", ["Remote Model Access Configuration",
        "Automatically generated by " + CM_BUILD_CFG_SCRIPT,
        "DO NOT EDIT THIS FILE!"])
    extqu.setlong("BufferIdOffset", 2048)
    extqufn = ""
    for i in range(top.Count):
        mdl = top.Item(i)
        if not mdl.IsInApplication:
            continue
        prop = mdl.Properties.TryGetItem("Model location")
        if prop is None:
            continue
        mdldir = os.path.dirname(prop.Value)
        if ModelContains(mdl, CM4SL_BLOCK_NAME):
            if PROJECT_DIR != "":
                cfgdir = os.path.join(PROJECT_DIR, "Data", "Config")
            else:
                cfgdir = os.path.join(os.path.dirname(mdldir), "Data", "Config")
            extqufn = os.path.join(cfgdir, "ExternalQuantities.info")
            continue
        exttab.append(mdl.Name + " " + str(i+1))
        rmaqu = infofiles.IFile()
        try:
            rmaqu.read(os.path.join(mdldir, mdl.Name + "_rma.info"))
            dstkey = mdl.Name + ".Model"
            extqu.setstr(dstkey, mdl.Name)
            extqu.addlinebefore("", dstkey)
            for key in ["Version"]:
                data = rmaqu.getstr(key)
                if data is not None:
                    extqu.setstr(mdl.Name + "." + key, data)
            for key in ("Description", "DDict", "Read", "Write", "Param"):
                data = rmaqu.gettxt(key)
                if data is not None:
                    extqu.settxt(mdl.Name + "." + key, data)
        except:
            continue
        finally:
            del rmaqu
    extqu.settxt("Topology", exttab)
    extqu.movekeybehind("Topology", "BufferIdOffset")
    if extqufn != "":
        extqu.write(extqufn)
    del extqu


def RMA_ExtQuants_Create_Txt(app):
    global PROJECT_DIR

    top = app.Components.Item("ModelTopology")
    exttab = []
    extmdls = []
    extsrcs = []
    extqufn = ""
    for i in range(top.Count):
        mdl = top.Item(i)
        if not mdl.IsInApplication:
            continue
        prop = mdl.Properties.TryGetItem("Model location")
        if prop is None:
            continue
        mdldir = os.path.dirname(prop.Value)
        if ModelContains(mdl, CM4SL_BLOCK_NAME):
            if PROJECT_DIR != "":
                cfgdir = os.path.join(PROJECT_DIR, "Data", "Config")
            else:
                cfgdir = os.path.join(os.path.dirname(mdldir), "Data", "Config")
            extqufn = os.path.join(cfgdir, "ExternalQuantities.info")
            continue
        exttab.append(mdl.Name + " " + str(i+1))
        extmdls.append(mdl.Name)
        extsrcs.append(os.path.join(mdldir, mdl.Name + "_rma.info"))

    if len(exttab) <= 0 or len(extqufn) <= 0:
        return
    extfile = open(extqufn, "w")
    extfile.writelines([
        "FileIdent = CarMaker-ExternalQuantities 1\n",
        "Description:\n",
        "\tRemote Model Access Configuration\n"
        "\tAutomatically generated by " + CM_BUILD_CFG_SCRIPT + "\n",
        "\tDO NOT EDIT THIS FILE!\n",
        "BufferIdOffset = 2048\n",
        "Topology:\n"])
    for i in range(len(exttab)):
        extfile.write("\t" + exttab[i] + "\n")
    for i in range(len(extmdls)):
        mdl = extmdls[i]
        src = extsrcs[i]
        srcfile = open(extsrcs[i], "r")
        lineno  = 0
        extfile.write("\n")
        for line in srcfile:
            lineno = lineno + 1
            if lineno == 1 and line.startswith("#INFOFILE"):
                continue
            if line.startswith("FileIdent"):
                continue
            if re.search(r"^[^\s=]+\s*=.*", line) is not None:
                extfile.write(mdl + "." + line)
            elif re.search(r"^[^\s:]+\s*:$", line) is not None:
                extfile.write(mdl + "." + line)
            else:
                extfile.write(line)
        srcfile.close()
    extfile.close()


# Main Program
if __name__ == '__main__':
    CreateProjectEventHandler()
    if AppEvH is None:
        MakeBuildConfig()
