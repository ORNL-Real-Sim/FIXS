// Prints the EgoSetup ConfigHelper.cpp resolves from a yaml, for tests/VirEnv/test_ego_setup_parity.py.
//
//   ego_dump.exe <config.yaml>

#include "ConfigHelper.h"

#include <cstdio>

int main(int argc, char** argv) {
	if (argc < 2) {
		fprintf(stderr, "usage: ego_dump <config.yaml>\n");
		return 2;
	}
	ConfigHelper c;
	c.getConfig(argv[1]);
	const EgoSetup_t& e = c.EgoSetup;
	printf("EGO|%s|%s|%s|%s|%s\n", e.Id.c_str(), e.Type.c_str(), e.Dynamics.c_str(),
		e.ActuationSource.c_str(), e.Controller.c_str());
	return 0;
}
