
#include "VirEnvHelper.h"
#include "VirEnv_Wrapper.h"

extern "C" {


	VirEnvHelper* newVirEnvHelper() {
		return new VirEnvHelper();
	}

	void VirEnv_shutdown(VirEnvHelper* VirEnv_c) {
		VirEnv_c->shutdown();
	};

	void VirEnv_initialization(VirEnvHelper* VirEnv_c, const char* configPath, const char* signalTablePath) {
        VirEnv_c->CM_Log("RealSim start initialization\n");

		#ifdef RS_DSPACE
			FILE * fp = fopen(configPath, "r");
			if (fp == NULL) {
				VirEnv_c->CM_Log("RealSim: Read RealSimCarMakerConfig.txt failed\n");
				return;
			}else{
				VirEnv_c->CM_Log("RealSim: Start readConfig\n");
				VirEnv_readConfig(VirEnv_c, fp);
			}
			fclose(fp);


			const char* errorMsg;
			if (VirEnv_c->initialization(&errorMsg, configPath, VirEnv_c->Config_s.SignalTableFilename.c_str()) < 0) {
				VirEnv_c->CM_LogErrF(errorMsg);
				VirEnv_c->CM_Log("RealSim error initialization \n");
			}
		#else
			const char* errorMsg;
			if (VirEnv_c->initialization(&errorMsg, configPath, signalTablePath) < 0) {
				VirEnv_c->CM_LogErrF(errorMsg);
				VirEnv_c->CM_Log("RealSim error initialization \n");
			}
		#endif
	
	}

	void VirEnv_runStep(VirEnvHelper* VirEnv_c, double simTime) {
		const char* errorMsg;
		if (VirEnv_c->runStep(SimCore.Time, &errorMsg) < 0) {
			// A SOCKET failure here means the traffic-simulator side (TrafficLayer
			// / VISSIM) closed the co-sim connection: the normal end-of-run
			// (TrafficLayer finishes its tick budget and closes its sockets) or the
			// user stopping a peer (TrafficLayer / VISSIM / CarMaker) first. In
			// every such case it is a GRACEFUL end of the co-simulation, so end the
			// CarMaker run cleanly (SCState_End -> SIM_END) instead of CM_LogErrF,
			// which is EC_General and makes CarMaker SIM_ABORT (the spurious
			// "Receive from traffic simulator failed" abort at the run boundary).
			// Genuine data faults (traffic-object init/update) keep the abort.
			const bool peerClosed = errorMsg != NULL &&
				(strstr(errorMsg, "Receive from traffic simulator") != NULL ||
				 strstr(errorMsg, "Send ego states") != NULL);
			if (peerClosed) {
				VirEnv_c->CM_Log("RealSim: traffic simulator closed -- ending run cleanly (SIM_END)\n");
				SimCore_State_Set(SCState_End);
			} else {
				VirEnv_c->CM_LogErrF(errorMsg);
				VirEnv_c->CM_Log("RealSim error run step \n");
			}
		}
	}

	int VirEnv_isVeryFirstStep(VirEnvHelper* VirEnv_c){
		return VirEnv_c->veryFirstStep;
	}

#ifdef RS_DSPACE
	int VirEnv_readConfig(VirEnvHelper* VirEnv_c, FILE *fp){
		char buf[1024];
		while (fgets(buf, sizeof buf, fp)) {
			char name[1024];
			char valChar[1024];
			name[0] = '\0';
			valChar[0] = '\0';
			int nRead = sscanf(buf, "%[^=]=%s",name, valChar);
			if (nRead > 0){
				VirEnv_getConfig(VirEnv_c, name, valChar);
			}
		}
		return 1;
	}

	int VirEnv_getConfig(VirEnvHelper* VirEnv_c, char* name, char* valChar){
		
		if (strcmp(name, "VehicleMessageField") == 0) {
			// Returns first message field
			char* token = strtok(valChar, ",");

			// Keep printing tokens while one of the
			// delimiters present in str[].
			while (token != NULL) {
				VirEnv_c->Config_s.VehicleMessageField_v.push_back(token);
				token = strtok(NULL, ",");
			}

		}
		if (strcmp(name, "EnableCosimulation") == 0) {
			if (strcmp(valChar, "True")==0){
				VirEnv_c->Config_s.EnableCosimulation = 1;
			}else{
				VirEnv_c->Config_s.EnableCosimulation = 0;
			}
		}
		if (strcmp(name, "EnableEgoSimulink") == 0) {
			if (strcmp(valChar, "True")==0){
				VirEnv_c->Config_s.EnableEgoSimulink = 1;
			}else{
				VirEnv_c->Config_s.EnableEgoSimulink = 0;
			}			
		}
		if (strcmp(name, "EgoId") == 0) {
			VirEnv_c->Config_s.EgoId = valChar;
		}
		if (strcmp(name, "EgoType") == 0) {
			VirEnv_c->Config_s.EgoType = valChar;
		}
		if (strcmp(name, "TrafficLayerIP") == 0) {
			VirEnv_c->Config_s.TrafficLayerIP = valChar;
		}
		if (strcmp(name, "CarMakerPort") == 0) {
			VirEnv_c->Config_s.CarMakerPort = atoi(valChar);
		}
		if (strcmp(name, "TrafficRefreshRate") == 0) {
			VirEnv_c->Config_s.TrafficRefreshRate = atof(valChar);
		}
		if (strcmp(name, "TrafficSignalPort") == 0) {
			VirEnv_c->Config_s.TrafficSignalPort = atoi(valChar);
		}
		if (strcmp(name, "SynchronizeTrafficSignal") == 0) {
			if (strcmp(valChar, "True") == 0) {
				VirEnv_c->Config_s.SynchronizeTrafficSignal = 1;
			}
			else {
				VirEnv_c->Config_s.SynchronizeTrafficSignal = 0;
			}
		}
		if (strcmp(name, "SignalTableFilename") == 0) {
			VirEnv_c->Config_s.SignalTableFilename = valChar;
		}
		return 0;
	}
#endif

}