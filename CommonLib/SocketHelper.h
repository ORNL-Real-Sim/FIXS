#pragma once

// will use event driven socket in the future
//#include <event2/event.h>

//#include <event2/bufferevent.h>
//#include <event2/util.h>
//#include <event2/event.h>

#include <cstdint>   // #65: uint8_t in sendData(); don't rely on a transitive include
#include "MsgHelper.h"

#ifndef WIN32
	#include <sys/types.h>
	#include <sys/socket.h>
	#include <netinet/in.h>
	#include <netinet/tcp.h>
	#include <arpa/inet.h>
	
	#include <ostream>
	#include <fstream>

	#include <stdio.h>
	#include <stdlib.h>
	#include <unistd.h>
	#include <string.h>
	#include <fcntl.h>
	#include <cstring>
	#include <string>

	#include <sys/select.h>
	
	#include <cmath>
	
	#include <cstdio>
	#include <cstdlib>
	
	#include <climits>

	#define SOCKET_ERROR (-1)
//extern int close(int __fildes);

	// NOTE: SocketHelper is generic transport (shared by TrafficLayer and the
	// SDK-free VirtualEnvironment core). It must NEVER pull a simulator SDK. The
	// BSD socket set above is sufficient on the RT/dSPACE (non-WIN32) target;
	// <CarMaker.h> was a leftover and is removed so the core stays CI-buildable.
	// Anything needing CarMaker belongs in the CarMaker backend TU, not here.

#else
	#include <winsock2.h>
	#include <ws2tcpip.h>
	#include <windows.h>
	#include <tchar.h>

	#pragma comment (lib, "Ws2_32.lib")
	#pragma comment (lib, "Mswsock.lib")
	#pragma comment (lib, "AdvApi32.lib")

// NOTE: RS_DEBUG must NOT pull <CarMaker.h> here. SocketHelper is shared with
// TrafficLayer, whose build has no CarMaker SDK on its include path -- so the
// RS_DEBUG breadcrumbs in SocketHelper.cpp use std::cout, not CarMaker's Log().

#endif

#include <iostream>
#include <fstream>
#include <sstream>

#include <unordered_map>

#define MAXPENDING 5    /* Maximum outstanding connection requests */

// RealSim header size and each message header size
#define MSG_HEADER_SIZE 9
#define MSG_EACH_HEADER_SIZE 3

// #87: size of the RECEIVE scratch buffer -- one record, never a whole message, so
// message size is unbounded while memory stays fixed (which is also what lets the
// Simulink / dSPACE targets take part without dynamic allocation).
//
// A VEHICLE record has a provable ceiling of 1888 B:
//     3 (uint16 size + uint8 type)
//   + 7 string fields x (1 length byte + 255 max chars) = 1792
//   + 93 bytes of numeric fields
// Strings are length-prefixed with a uint8, so 255 is a hard per-field ceiling.
//
// A DETECTOR record has NO such ceiling: TlsDetector_t carries a
// vector<DetectorData_t>, so its size grows with the detector count at an
// intersection and is bounded only by the uint16 size field (65535). 8192 covers a
// few hundred realistically-named detectors; a record above it is refused loudly in
// recvData rather than overflowing the buffer (the old code passed the declared size
// straight to recv() with a 1024-byte buffer, so anything over 1024 already
// corrupted the stack silently).
//
// Not sized to 65535: that buffer is a local in recvData, and the CarMaker / dSPACE
// real-time tasks run on small stacks. Raise this if a scenario legitimately needs it.
#define MAX_RECORD_SIZE 8192

// #87: send-side chunk buffer. Must be >= MAX_RECORD_SIZE so one record always fits
// after a flush. Larger just means fewer send() syscalls -- it does NOT cap the
// message size, which is streamed across as many chunks as needed.
//
// MAX_RECORD_SIZE, not this, is the wire contract: sendData refuses to EMIT a record
// above MAX_RECORD_SIZE and recvData refuses to ACCEPT one, so both ends agree on
// what is representable. Sizing the send guard off TX_CHUNK_SIZE instead would let a
// C++ sender emit a record that a C++ receiver then rejects by dropping the link.
#define TX_CHUNK_SIZE 16384


class SocketHelper
{

public:
    // constructor
    SocketHelper();

	// different ways to initialize socket
	void socketSetup(std::vector <std::string> SERVERADDR_UserInput, std::vector <int> SERVERPORT_UserInput);
	void socketSetup(std::vector <int> selfServerPortUserInput);
	void socketSetup(std::vector <std::string> SERVERADDR_UserInput, std::vector <int> SERVERPORT_UserInput, std::vector <int> selfServerPortUserInput);

	void socketReset();

	void enableWaitClientTrigger();
	void disableWaitClientTrigger();
	void enableServerTrigger();
	void disableServerTrigger();
	void disableServer();
	void disableClient();

	int initConnection(std::string errorLogName="");

	// Warm-up support (#86). When set before initConnection(), the sockets are
	// still created, bound and listen()ed -- so a client's connect() succeeds
	// exactly as before and lands in the backlog -- but the blocking accept loop
	// is NOT run. The host calls acceptClients() later, when its warm-up ends.
	//
	// The point is not the accept itself (a client that connects early simply
	// blocks on its first recv either way); it is that TrafficLayer no longer
	// waits for every client to exist before it starts warming up, so CarMaker's
	// and CARLA's start-up overlaps the warm-up instead of queueing ahead of it.
	//
	// SCOPE: this defers the INBOUND half only -- accepting the clients that
	// connect to us. It does NOT defer the OUTBOUND half, the ENABLE_SERVER block
	// that connects to other servers, which still runs inside initConnection
	// before this returns.
	//
	// That is enough today because TrafficLayer is only ever a listener: both of
	// its socketSetup calls use the selfServerPort-only overload, which sets
	// NSERVER = 0 / ENABLE_SERVER = 0, and every peer (application layer, XIL /
	// CarMaker lib, VirCarlaEnv, VISSIM DriverModel) connects in to it. The
	// outbound path is what those other components use to reach TrafficLayer, and
	// none of them runs a warm-up.
	//
	// If a host ever needs BOTH roles and a warm-up, the outbound connects have to
	// be deferred too -- otherwise it still blocks waiting for its own server to
	// exist and the warm-up gains nothing on that half of its peers.
	bool DeferAcceptClients = false;

	// The blocking "wait for all clients" accept loop. Run automatically by
	// initConnection() unless DeferAcceptClients is set; call it directly when it
	// was deferred. Returns 0 on success, -1 on a socket error.
	int acceptClients(std::string errorLogName="");

	// RS_DEBUG master-log target. The host assigns it (TrafficLayer sets it to its
	// RealSim_tmp/TrafficLayer_<timestamp>.log). When empty -- e.g. the VirtualEnvironment
	// .lib running inside CarMaker -- RS_DEBUG breadcrumbs fall back to
	// RealSim_tmp/RealSim_debug.log so there is always a persistent log on disk.
	std::string MasterLogName = "";
#ifdef RS_DEBUG
	void rsDebugLog(const char* msg);
#endif

	void socketShutdown();

	// recv data 
	int recvData(int sock, int* simState, float* simTime, MsgHelper& Msg_c);
		
	// send data
	int sendData(int sock, int iClient, float simTimeSend, uint8_t simStateSend, MsgHelper Msg_c);


	void printSocketErrorMessage(int errorCode);

// below should be converted to private in the future
//private:

	// #87: send-side chunk buffer. A member so it is reused rather than re-established
	// on every sendData() call, and so its size is visible in one place.
	//
	// It does NOT move the cost off the stack: SocketHelper is itself a stack object at
	// several call sites (TrafficHelper.cpp:894, :1844, DSProxyMode.cpp:193), so there
	// the buffer lives on their stack instead. Together with the MAX_RECORD_SIZE local
	// in recvData this raises stack use, which is worth knowing before either constant
	// is increased on a real-time target.
	//
	// Never holds a whole message -- only one chunk at a time.
	char txBuf[TX_CHUNK_SIZE];

	int NSERVER = 0;
	int NCLIENT = 0;

	int N_ACT_CLIENT = 0; // actual clients exculde VISSIM

	bool ENABLE_CLIENT = 0;
	bool ENABLE_SERVER = 1;

	bool ENABLE_WAIT_CLIENT_TRIGGER = 1;
	bool ENABLE_SERVER_TRIGGER = 1;

	std::vector <std::string> SERVERADDR;
	std::vector <int> SERVERPORT;

	std::vector <int> serverSock;
	std::vector <int> clientSock;

	std::vector <unsigned short> selfServerPort;

	// Server socket initialize
	std::vector <struct sockaddr_in> serverAddr;
	std::vector <int> serverAddrLen;
	std::vector <int> sendServerByte;

	// Client socket initialize
	std::vector <int> ClientConnected;
	int AllClientConnected = 0;

	std::vector <int> ClientRequested;
	int AllClientRequested = 0;


	int ClientRecvStatus = 0;

	//
	std::vector <int> selfServerSock;                    /* Socket descriptor for server */
	std::unordered_map <int, std::string> sockName_um;

	std::vector <struct sockaddr_in> selfServerAddr; /* Local a ddress */
	std::vector <struct sockaddr_in> clientAddr; /* Client address */
#ifdef WIN32
	std::vector <int> clientAddrLen;                  /* Length of client address data structure */
#else
	// #65: every POSIX target -- dSPACE/RT and plain Linux alike -- must use
	// socklen_t here, because that is what accept()'s third parameter points to.
	// The plain-POSIX branch previously said size_t, which no compiler had ever
	// checked: it is 64-bit where socklen_t is 32-bit, so it could not compile.
	std::vector <socklen_t> clientAddrLen;            /* Length of client address data structure */
#endif
	std::vector <int> sendClientByte;
	std::vector <int> recvClientMsgSize;                    /* Size of received message */
	 

};

