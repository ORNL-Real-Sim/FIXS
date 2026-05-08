classdef RealSimMsgClass < handle
    % RealSimMsgClass Basis Class that contains functions to support pack and depack 

    properties(Constant)
        % Maximum number of route edges stored in the fixed-size Simulink Bus array.
        % Must match the second dimension of routeEdges in VehicleDataEmpty and the Bus definition.
        ROUTE_MAX_EDGES = 20;
        % Maximum byte length of a single edge ID string (e.g. "-2081").
        % Must match the first dimension of routeEdges in VehicleDataEmpty and the Bus definition.
        ROUTE_MAX_EDGE_LEN = 50;
    end

    
    properties
        VehicleMessageFieldDefInputVec = zeros(1, 30); % Vector of 0,1 to select which message field will be transmitted
        
    end
        
    properties(Access = protected)
        
        % create a bunches of flags of all possible vehicle message field so
        % that in each simulation step we do not need to search for string,
        % but simply check the flag of each data field. 
        % ***could use byte bit later
        VehicleMessageFieldDef = struct('id', 0, 'type', 0, 'vehicleClass', 0, 'speed', 0, 'acceleration', 0, ...
            'positionX', 0, 'positionY', 0, 'positionZ', 0, 'heading', 0, 'color', 0, ...
            'linkId', 0, 'laneId', 0, 'distanceTravel', 0, 'speedDesired', 0, 'accelerationDesired', 0, ...
            'hasPrecedingVehicle', 0, 'precedingVehicleId', 0, 'precedingVehicleDistance', 0, 'precedingVehicleSpeed', 0, ...
            'signalLightId', 0, 'signalLightHeadId', 0, 'signalLightDistance', 0, 'signalLightColor', 0, 'speedLimit', 0, 'speedLimitNext', 0, 'speedLimitChangeDistance', 0, ...
            'linkIdNext', 0, 'grade', 0, 'activeLaneChange', 0, 'routeEdges', 0);

        % DO NOT change. These are predetermined message header size
        % specified for Real-Sim
        msgHeaderSize = 9;
        msgEachHeaderSize = 3;
        
        % define data types identifier
        vehMsgIdentifer = 1;
        
        % debugging function
        enableDebug = 0;
        
        VehicleDataEmpty = struct('id', uint8(zeros(50,1)), 'idLength', 0, 'type', uint8(zeros(50,1)), 'typeLength', 0, ...
                    'vehicleClass', uint8(zeros(50,1)), 'vehicleClassLength', 0, ...
                    'speed', 0, 'acceleration', 0, 'positionX', 0, 'positionY', 0, 'positionZ', 0, ...
                    'heading', 0, 'color', 0, 'linkId', uint8(zeros(50,1)), 'linkIdLength', 0, 'laneId', 0, ...
                    'distanceTravel', 0, 'speedDesired', 0, 'accelerationDesired', 0, ...
                    'hasPrecedingVehicle', 0, 'precedingVehicleId', uint8(zeros(50,1)), 'precedingVehicleIdLength', 0, 'precedingVehicleDistance', 0, 'precedingVehicleSpeed', 0, ...
                    'signalLightId', uint8(zeros(50,1)), 'signalLightIdLength', 0, 'signalLightHeadId', 0, 'signalLightDistance', 0, 'signalLightColor', 0, ...
                    'speedLimit', 0, 'speedLimitNext', 0, 'speedLimitChangeDistance', 0, ...
                    'linkIdNext', uint8(zeros(50,1)), 'linkIdNextLength', 0, 'grade', 0, 'activeLaneChange', 0, 'routeEdgesCount', uint16(0), ...
                    'routeEdgesLengths', uint8(zeros(20,1)), ...
                    'routeEdges', uint8(zeros(50, 20)));
    
    end
    
    methods
        % Constructor
        function obj = RealSimMsgClass(varargin)
            % Support name-value pair arguments when constructing object
%             setProperties(obj,nargin,varargin{:})
%             coder.allowpcode('plain')

        end
    end
    
    methods(Access = public)
               
        function VehMsgDefVec2VehMsgDef(obj)
        % convert VehicleMessageFieldDefInputVec to VehicleMessageFieldDef
            VehMsgAllList = {'id', 'type', 'vehicleClass', 'speed', 'acceleration', ...
                'positionX', 'positionY', 'positionZ', 'heading', 'color', ...
                'linkId', 'laneId', 'distanceTravel', 'speedDesired', 'accelerationDesired', ...
                'hasPrecedingVehicle', 'precedingVehicleId', 'precedingVehicleDistance', 'precedingVehicleSpeed', ...
                'signalLightId', 'signalLightHeadId', 'signalLightDistance', 'signalLightColor', 'speedLimit', 'speedLimitNext', 'speedLimitChangeDistance', ...
                'linkIdNext', 'grade', 'activeLaneChange', 'routeEdges'};

            for i = 1:numel(VehMsgAllList)
                obj.VehicleMessageFieldDef.(VehMsgAllList{i}) = obj.VehicleMessageFieldDefInputVec(i);               
            end

        end
        
        
        function VehData = depackVehData(obj, VehData, ByteData)
            
            iByte = 1;
            
            if obj.VehicleMessageFieldDef.('id')
                [str, strLen, iByte, uint8Arr] = obj.depackString(ByteData, iByte);
                VehData.('id') = uint8Arr; 
                VehData.('idLength') = strLen;
            end
            if obj.VehicleMessageFieldDef.('type')
                [str, strLen, iByte, uint8Arr] = obj.depackString(ByteData, iByte);
                VehData.('type') = uint8Arr; 
                VehData.('typeLength') = strLen;
            end
            if obj.VehicleMessageFieldDef.('vehicleClass')
                [str, strLen, iByte, uint8Arr] = obj.depackString(ByteData, iByte);
                VehData.('vehicleClass') = uint8Arr; 
                VehData.('vehicleClassLength') = strLen;
            end
            if obj.VehicleMessageFieldDef.('speed')
                VehData.('speed') = double(typecast(ByteData(iByte:iByte+3),'single'));
                iByte = iByte+4;
            end
            if obj.VehicleMessageFieldDef.('acceleration')
                VehData.('acceleration') = double(typecast(ByteData(iByte:iByte+3),'single'));
                iByte = iByte+4;
            end
            if obj.VehicleMessageFieldDef.('positionX')
                VehData.('positionX') = double(typecast(ByteData(iByte:iByte+3),'single'));
                iByte = iByte+4;
            end
            if obj.VehicleMessageFieldDef.('positionY')
                VehData.('positionY') = double(typecast(ByteData(iByte:iByte+3),'single'));
                iByte = iByte+4;
            end
            if obj.VehicleMessageFieldDef.('positionZ')
                VehData.('positionZ') = double(typecast(ByteData(iByte:iByte+3),'single'));
                iByte = iByte+4;
            end
            if obj.VehicleMessageFieldDef.('heading')
                VehData.('heading') = double(typecast(ByteData(iByte:iByte+3),'single'));
                iByte = iByte+4;
            end
            if obj.VehicleMessageFieldDef.('color')
                VehData.('color') = double(typecast(ByteData(iByte:iByte+3),'uint32'));
                iByte = iByte+4;
            end
            if obj.VehicleMessageFieldDef.('linkId')
                [str, strLen, iByte, uint8Arr] = obj.depackString(ByteData, iByte);
                VehData.('linkId') = uint8Arr; 
                VehData.('linkIdLength') = strLen;		   
            end
            if obj.VehicleMessageFieldDef.('laneId')
                VehData.('laneId') = double(typecast(ByteData(iByte:iByte+3),'int32'));
                iByte = iByte+4;
            end
            if obj.VehicleMessageFieldDef.('distanceTravel')
                VehData.('distanceTravel') = double(typecast(ByteData(iByte:iByte+3),'single'));
                iByte = iByte+4;
            end
            if obj.VehicleMessageFieldDef.('speedDesired')
                VehData.('speedDesired') = double(typecast(ByteData(iByte:iByte+3),'single'));
                iByte = iByte+4;
            end
            if obj.VehicleMessageFieldDef.('accelerationDesired')
                VehData.('accelerationDesired') = double(typecast(ByteData(iByte:iByte+3),'single'));
                iByte = iByte+4;
            end
            if obj.VehicleMessageFieldDef.('hasPrecedingVehicle')
                VehData.('hasPrecedingVehicle') = double(typecast(ByteData(iByte),'int8'));
                iByte = iByte+1;
            end
            if obj.VehicleMessageFieldDef.('precedingVehicleId')
                [str, strLen, iByte, uint8Arr] = obj.depackString(ByteData, iByte);
                VehData.('precedingVehicleId') = uint8Arr; 
                VehData.('precedingVehicleIdLength') = strLen;	
            end
            if obj.VehicleMessageFieldDef.('precedingVehicleDistance')
                VehData.('precedingVehicleDistance') = double(typecast(ByteData(iByte:iByte+3),'single'));
                iByte = iByte+4;
            end
            if obj.VehicleMessageFieldDef.('precedingVehicleSpeed')
                VehData.('precedingVehicleSpeed') = double(typecast(ByteData(iByte:iByte+3),'single'));
                iByte = iByte+4;
            end
            if obj.VehicleMessageFieldDef.('signalLightId')
                [str, strLen, iByte, uint8Arr] = obj.depackString(ByteData, iByte);
                VehData.('signalLightId') = uint8Arr; 
                VehData.('signalLightIdLength') = strLen;
            end
            if obj.VehicleMessageFieldDef.('signalLightHeadId')
                VehData.('signalLightHeadId') = double(typecast(ByteData(iByte:iByte+3),'int32'));
                iByte = iByte+4;
            end
            if obj.VehicleMessageFieldDef.('signalLightDistance')
                VehData.('signalLightDistance') = double(typecast(ByteData(iByte:iByte+3),'single'));
                iByte = iByte+4;
            end
            if obj.VehicleMessageFieldDef.('signalLightColor')
                VehData.('signalLightColor') = double(typecast(ByteData(iByte),'int8'));
                iByte = iByte+1;
            end
            if obj.VehicleMessageFieldDef.('speedLimit')
                VehData.('speedLimit') = double(typecast(ByteData(iByte:iByte+3),'single'));
                iByte = iByte+4;
            end
            if obj.VehicleMessageFieldDef.('speedLimitNext')
                VehData.('speedLimitNext') = double(typecast(ByteData(iByte:iByte+3),'single'));
                iByte = iByte+4;
            end
            if obj.VehicleMessageFieldDef.('speedLimitChangeDistance')
                VehData.('speedLimitChangeDistance') = double(typecast(ByteData(iByte:iByte+3),'single'));
                iByte = iByte+4;
            end
            if obj.VehicleMessageFieldDef.('linkIdNext')
                [str, strLen, iByte, uint8Arr] = obj.depackString(ByteData, iByte);
                VehData.('linkIdNext') = uint8Arr; %str(1:strLen);
                VehData.('linkIdNextLength') = strLen;	
            end
            if obj.VehicleMessageFieldDef.('grade')
                VehData.('grade') = double(typecast(ByteData(iByte:iByte+3),'single'));
                iByte = iByte+4;
            end
            if obj.VehicleMessageFieldDef.('activeLaneChange')
                VehData.('activeLaneChange') = double(typecast(ByteData(iByte),'int8'));
                iByte = iByte+1;
            end
            if obj.VehicleMessageFieldDef.('routeEdges')
                % Read total number of edges transmitted
                routeSize = double(typecast(ByteData(iByte:iByte+1), 'uint16'));
                iByte = iByte + 2;

                VehData.('routeEdgesCount') = uint16(routeSize);
                VehData.('routeEdges') = uint8(zeros(obj.ROUTE_MAX_EDGE_LEN, obj.ROUTE_MAX_EDGES));
                VehData.('routeEdgesLengths') = uint8(zeros(obj.ROUTE_MAX_EDGES, 1));

                % Decode each edge; only store up to ROUTE_MAX_EDGES
                for k = 1:routeSize
                    strLen = double(typecast(ByteData(iByte:iByte+1), 'uint16'));
                    iByte = iByte + 2;

                    if k <= obj.ROUTE_MAX_EDGES
                        % Clamp copy length to the fixed row size
                        copyLen = min(strLen, obj.ROUTE_MAX_EDGE_LEN);
                        VehData.('routeEdges')(1:copyLen, k) = ByteData(iByte:iByte+copyLen-1);
                        VehData.('routeEdgesLengths')(k) = uint8(copyLen);
                    end
                    % Always advance iByte by the full transmitted length
                    iByte = iByte + strLen;
                end
            end
        end
        
        
        function [str, strLen, iByte, uint8Arr] = depackString(obj, ByteData, iByte)
            strLen = double(typecast(ByteData(iByte),'uint8'));
            if strLen > 50
                error('max characters of id, linkId, type, precedingVehicleId, linkIdNext must be smaller than %d', 50);
            end
            iByte = iByte+1;
            % initialize with size 50 
            str = blanks(50);
            uint8Arr = uint8(zeros(50,1));
            for i = 1:round(strLen)
%                 str = [str, ByteData(iByte)];
                str(i) = ByteData(iByte);
                uint8Arr(i) = ByteData(iByte);
                iByte = iByte+1;
            end         
        end
        
        function [ByteData, nMsgSize] = packVehData(obj, simState, t, ByteData, VehData)
            
            routeEdgesSize = 0;
            if obj.VehicleMessageFieldDef.('routeEdges')
                count = double(VehData.('routeEdgesCount'));
                routeEdgesSize = 2; % uint16 for total edge count
                for k = 1:min(count, obj.ROUTE_MAX_EDGES)
                    % Use the stored per-edge length instead of scanning for a null terminator
                    edgeLen = double(VehData.('routeEdgesLengths')(k));
                    routeEdgesSize = routeEdgesSize + 2 + edgeLen; % uint16 length prefix + data
                end
            end



            nMsgSize = round( obj.msgHeaderSize + obj.msgEachHeaderSize  ...
                + obj.VehicleMessageFieldDef.('id')*(1 + VehData.('idLength')) ...
                + obj.VehicleMessageFieldDef.('type')*(1 + VehData.('typeLength')) ...
                + obj.VehicleMessageFieldDef.('vehicleClass')*(1 + VehData.('vehicleClassLength')) ...
                + obj.VehicleMessageFieldDef.('speed')*(4) ... % speed
                + obj.VehicleMessageFieldDef.('acceleration')*(4) ... % acceleration
                + obj.VehicleMessageFieldDef.('positionX')*(4) ... % positionX
                + obj.VehicleMessageFieldDef.('positionY')*(4) ... % positionY
                + obj.VehicleMessageFieldDef.('positionZ')*(4) ... % positionZ
                + obj.VehicleMessageFieldDef.('heading')*(4) ... % heading
                + obj.VehicleMessageFieldDef.('color')*(4) ... % color
                + obj.VehicleMessageFieldDef.('linkId')*(1 + VehData.('linkIdLength')) ... % linkId
                + obj.VehicleMessageFieldDef.('laneId')*(4) ... % laneId
                + obj.VehicleMessageFieldDef.('distanceTravel')*(4) ... % distanceTravel
                + obj.VehicleMessageFieldDef.('speedDesired')*(4) ... % speedDesired
                + obj.VehicleMessageFieldDef.('accelerationDesired')*(4) ... % accelerationDesired
                + obj.VehicleMessageFieldDef.('hasPrecedingVehicle')*(1) ... % hasPrecedingVehicle
                + obj.VehicleMessageFieldDef.('precedingVehicleId')*(1 + VehData.('precedingVehicleIdLength')) ... % precedingVehicleId
                + obj.VehicleMessageFieldDef.('precedingVehicleDistance')*(4) ... % precedingVehicleDistance
                + obj.VehicleMessageFieldDef.('precedingVehicleSpeed')*(4) ... % precedingVehicleSpeed
                + obj.VehicleMessageFieldDef.('signalLightId')*(1 + VehData.('signalLightIdLength')) ... % signalLightId
                + obj.VehicleMessageFieldDef.('signalLightHeadId')*(4) ... % signalLightHeadId
                + obj.VehicleMessageFieldDef.('signalLightDistance')*(4) ... % signalLightDistance
                + obj.VehicleMessageFieldDef.('signalLightColor')*(1) ... % signalLightColor
                + obj.VehicleMessageFieldDef.('speedLimit')*(4) ... % speedLimit
                + obj.VehicleMessageFieldDef.('speedLimitNext')*(4) ... % speedLimitNext
                + obj.VehicleMessageFieldDef.('speedLimitChangeDistance')*(4) ... % speedLimitChangeDistance
                + obj.VehicleMessageFieldDef.('linkIdNext')*(1 + VehData.('linkIdNextLength')) ... % linkIdNext
                + obj.VehicleMessageFieldDef.('grade')*(4) ... % grade
                + obj.VehicleMessageFieldDef.('activeLaneChange')*(1) ... % activeLaneChange
                + routeEdgesSize ... % routeEdges
                ) ;
            nVehMsgSize = round(nMsgSize - obj.msgHeaderSize);


            iByte = 1;
            
            tempUint8 = typecast(uint8(simState), 'uint8');
            ByteData(iByte) = tempUint8;
            iByte = iByte + 1;
            
            tempFloat = typecast(single(t), 'uint8');
            ByteData(iByte:iByte+3) = tempFloat;
            iByte = iByte + 4;
            
            tempUint32 = typecast(uint32(nMsgSize), 'uint8');
            ByteData(iByte:iByte+3) = tempUint32;
            iByte = iByte + 4;

            % message length
            tempUint16 = typecast(uint16(nVehMsgSize), 'uint8');
            ByteData(iByte:iByte+1) = tempUint16;
            iByte = iByte + 2;
            
            % message type
            tempUint8 = typecast(uint8(1), 'uint8');
            ByteData(iByte) = tempUint8;
            iByte = iByte + 1;

            if obj.VehicleMessageFieldDef.('id')          
                messageField = 'id'; 
                [ByteData, iByte] = obj.packString(ByteData, iByte, VehData, messageField);
            end
            
            if obj.VehicleMessageFieldDef.('type')
                messageField = 'type'; 
                [ByteData, iByte] = obj.packString(ByteData, iByte, VehData, messageField);
            end
            if obj.VehicleMessageFieldDef.('vehicleClass')
                messageField = 'vehicleClass';
                [ByteData, iByte] = obj.packString(ByteData, iByte, VehData, messageField);
            end
            if obj.VehicleMessageFieldDef.('speed')
                % speed
                tempFloat = typecast(single(VehData.('speed')), 'uint8');
                ByteData(iByte:iByte+3) = tempFloat;
                iByte = iByte + 4;
            end
            
            if obj.VehicleMessageFieldDef.('acceleration')
                % acceleration
                tempFloat = typecast(single(VehData.('acceleration')), 'uint8');
                ByteData(iByte:iByte+3) = tempFloat;
                iByte = iByte + 4;
            end
            
            if obj.VehicleMessageFieldDef.('positionX')
                % position X
                tempFloat = typecast(single(VehData.('positionX')), 'uint8');
                ByteData(iByte:iByte+3) = tempFloat;
                iByte = iByte + 4;
            end
            
            if obj.VehicleMessageFieldDef.('positionY')
                % position Y
                tempFloat = typecast(single(VehData.('positionY')), 'uint8');
                ByteData(iByte:iByte+3) = tempFloat;
                iByte = iByte + 4;
            end
            
            if obj.VehicleMessageFieldDef.('positionZ')
                % position Z
                tempFloat = typecast(single(VehData.('positionZ')), 'uint8');
                ByteData(iByte:iByte+3) = tempFloat;
                iByte = iByte + 4;
            end
            
            if obj.VehicleMessageFieldDef.('heading')
                % heading
                tempFloat = typecast(single(VehData.('heading')), 'uint8');
                ByteData(iByte:iByte+3) = tempFloat;
                iByte = iByte + 4;
            end
            
            if obj.VehicleMessageFieldDef.('color')
                % color
                tempUint32 = typecast(uint32(VehData.('color')), 'uint8');
                ByteData(iByte:iByte+3) = tempUint32;
                iByte = iByte + 4;
            end
            
            if obj.VehicleMessageFieldDef.('linkId')
                messageField = 'linkId'; 
                [ByteData, iByte] = obj.packString(ByteData, iByte, VehData, messageField);
            end
            
            if obj.VehicleMessageFieldDef.('laneId')
                % lane id
                tempInt32 = typecast(int32(VehData.('laneId')), 'uint8');
                ByteData(iByte:iByte+3) = tempInt32;
                iByte = iByte + 4;
            end
            
            if obj.VehicleMessageFieldDef.('distanceTravel')
                % distance travel
                tempFloat = typecast(single(VehData.('distanceTravel')), 'uint8');
                ByteData(iByte:iByte+3) = tempFloat;
                iByte = iByte + 4;
            end
                  
            if obj.VehicleMessageFieldDef.('speedDesired')
                % speedDesired
                tempFloat = typecast(single(VehData.('speedDesired')), 'uint8');
                ByteData(iByte:iByte+3) = tempFloat;
                iByte = iByte + 4;
            end
            
            if obj.VehicleMessageFieldDef.('accelerationDesired')
                % accelerationDesired
                tempFloat = typecast(single(VehData.('accelerationDesired')), 'uint8');
                ByteData(iByte:iByte+3) = tempFloat;
                iByte = iByte + 4;
            end
            
            if obj.VehicleMessageFieldDef.('hasPrecedingVehicle')
                % hasPrecedingVehicle
                tempInt8 = typecast(int8(VehData.('hasPrecedingVehicle')), 'uint8');
                ByteData(iByte) = tempInt8;
                iByte = iByte + 1;
            end
           
            if obj.VehicleMessageFieldDef.('precedingVehicleId')
                messageField = 'precedingVehicleId'; 
                [ByteData, iByte] = obj.packString(ByteData, iByte, VehData, messageField);
            end
           
            if obj.VehicleMessageFieldDef.('precedingVehicleDistance')
                % precedingVehicleDistance
                tempFloat = typecast(single(VehData.('precedingVehicleDistance')), 'uint8');
                ByteData(iByte:iByte+3) = tempFloat;
                iByte = iByte + 4;
            end
           
            if obj.VehicleMessageFieldDef.('precedingVehicleSpeed')
                % precedingVehicleSpeed
                tempFloat = typecast(single(VehData.('precedingVehicleSpeed')), 'uint8');
                ByteData(iByte:iByte+3) = tempFloat;
                iByte = iByte + 4;
            end
           
            if obj.VehicleMessageFieldDef.('signalLightId')
                messageField = 'signalLightId';
                [ByteData, iByte] = obj.packString(ByteData, iByte, VehData, messageField);
            end
            
            if obj.VehicleMessageFieldDef.('signalLightHeadId')
                % signalLightHeadId
                tempInt32 = typecast(int32(VehData.('signalLightHeadId')), 'uint8');
                ByteData(iByte:iByte+3) = tempInt32;
                iByte = iByte + 4;
            end
            
            if obj.VehicleMessageFieldDef.('signalLightDistance')
                % signalLightDistance
                tempFloat = typecast(single(VehData.('signalLightDistance')), 'uint8');
                ByteData(iByte:iByte+3) = tempFloat;
                iByte = iByte + 4;
            end
           
            if obj.VehicleMessageFieldDef.('signalLightColor')
                % signalLightColor
                tempInt8 = typecast(int8(VehData.('signalLightColor')), 'uint8');
                ByteData(iByte) = tempInt8;
                iByte = iByte + 1;
            end
           
            if obj.VehicleMessageFieldDef.('speedLimit')
                % speedLimit
                tempFloat = typecast(single(VehData.('speedLimit')), 'uint8');
                ByteData(iByte:iByte+3) = tempFloat;
                iByte = iByte + 4;
            end
           
            if obj.VehicleMessageFieldDef.('speedLimitNext')
                % speedLimitNext
                tempFloat = typecast(single(VehData.('speedLimitNext')), 'uint8');
                ByteData(iByte:iByte+3) = tempFloat;
                iByte = iByte + 4;
            end
           
            if obj.VehicleMessageFieldDef.('speedLimitChangeDistance')
                % speedLimitChangeDistance
                tempFloat = typecast(single(VehData.('speedLimitChangeDistance')), 'uint8');
                ByteData(iByte:iByte+3) = tempFloat;
                iByte = iByte + 4;
            end
           
            if obj.VehicleMessageFieldDef.('linkIdNext')
                messageField = 'linkIdNext'; 
                [ByteData, iByte] = obj.packString(ByteData, iByte, VehData, messageField);
            end
           
            if obj.VehicleMessageFieldDef.('grade')
                % grade
                tempFloat = typecast(single(VehData.('grade')), 'uint8');
                ByteData(iByte:iByte+3) = tempFloat;
                iByte = iByte + 4;
            end
           
            if obj.VehicleMessageFieldDef.('activeLaneChange')
                % activeLaneChange
                tempInt8 = typecast(int8(VehData.('activeLaneChange')), 'uint8');
                ByteData(iByte) = tempInt8;
                iByte = iByte + 1;
            end

            if obj.VehicleMessageFieldDef.('routeEdges')
                count = double(VehData.('routeEdgesCount'));
                % Write total number of edges
                tempUint16 = typecast(uint16(count), 'uint8');
                ByteData(iByte:iByte+1) = tempUint16;
                iByte = iByte + 2;

                for k = 1:min(count, obj.ROUTE_MAX_EDGES)
                    % Use stored length — no need to scan for null terminator
                    edgeLen = double(VehData.('routeEdgesLengths')(k));

                    tempUint16 = typecast(uint16(edgeLen), 'uint8');
                    ByteData(iByte:iByte+1) = tempUint16;
                    iByte = iByte + 2;

                    if edgeLen > 0
                        ByteData(iByte:iByte+edgeLen-1) = VehData.('routeEdges')(1:edgeLen, k);
                        iByte = iByte + edgeLen;
                    end
                end
            end

        end
        
        function [ByteData, iByte] = packString(obj, ByteData, iByte, VehData, messageField)
            strLen = VehData.([messageField,'Length']);
            % parser string length
            tempUint8 = typecast(uint8(strLen), 'uint8');
            ByteData(iByte) = tempUint8;
            iByte = iByte + 1;
            % parser string
            ByteData(iByte:(iByte+strLen-1)) = VehData.(messageField)(1:strLen);
            iByte = iByte + strLen;
        end
        
    %%% END OF PRIVATE METHODS    
    end 
    
   
%%% END OF CLASS
end
