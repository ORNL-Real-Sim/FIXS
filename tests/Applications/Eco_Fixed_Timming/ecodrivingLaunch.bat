
:: start SUMO 
start sumo-gui -c .\Test\chattCavMpr.sumocfg --remote-port 1337 --step-length 0.1 --netstate-dump .\Test\chattCavMpr.xml --netstate-dump.precision 10 --num-clients 2  --begin 28800 --end 33000
:: start TrafficLayer
start cmd /k .\TrafficLayer.exe -f .\ecodrivingConfig.yaml
