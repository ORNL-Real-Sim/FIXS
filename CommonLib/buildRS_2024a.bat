start cmd /d /e:on /k ""C:\Program Files\dSPACE ConfigurationDesk 2024-A (24.1)\CFD_vars.bat " & ""dsmake -f DsBuildLibrary_2024a.mk output_filename=RealSimDsLib_2024a source_files="SocketHelper.cpp MsgHelper.cpp VirEnvHelper.cpp VirEnv_Wrapper.cpp" custom_cpp_options="-std=c++11 -IC:\IPG\carmaker\win64-13.1.2\include -DDSRTLX -DRS_DSPACE -DRS_CAVE -DRS_DEBUG" target=Dsx86_32"" "

