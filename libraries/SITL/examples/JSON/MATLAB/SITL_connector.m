% this function handles all the UDP connection to SITL using Native MATLAB Java Sockets
% Bypasses the broken pnet MEX compiler requirements
function SITL_connector(state,init_function,physics_function,max_timestep)

% Clear any lingering Java sockets from the workspace if they exist
clear j_socket;

% init physics
state = init_function(state);

% Setup Native Java UDP Configurations
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress

local_port = 9002;
bytes_read = 4 + 4 + 16*2; % 44 bytes expected from ArduPilot

try
    % Bind to port 9002
    j_socket = DatagramSocket(local_port);
    j_socket.setSoTimeout(0); % Infinite block while waiting for data
    fprintf('Opened Native Java UDP socket on port %d\n', local_port);
catch ME
    error('Failed to open UDP socket. Ensure port %d is not bound by another process: %s', local_port, ME.message);
end

frame_time = tic;
frame_count = 0;
physics_time_s = 0;
last_SITL_frame = -1;
print_frame_count = 1000; % print the fps every x frames
connected = false;

% Pre-allocate receive buffer packet
receive_buffer = zeros(1, 1024, 'int8'); 
packet_in = DatagramPacket(receive_buffer, length(receive_buffer));

% Target tracking variables for returning data
target_ip = [];
target_port = [];

% Clean up socket if user hits Ctrl+C
cleanupObj = onCleanup(@() j_socket.close());

while true
    try
        % Wait and read packet from AP
        j_socket.receive(packet_in);
        
        % Extract raw byte stream
        raw_bytes = typecast(packet_in.getData(), 'uint8');
        actual_len = packet_in.getLength();
        
        if actual_len < bytes_read
            continue; % Malformed or short packet, drop it
        end
        
        % Read in binary data from AP (Emulating pnet stream reading)
        magic      = typecast(raw_bytes(1:2), 'uint16');
        frame_rate = double(typecast(raw_bytes(3:4), 'uint16'));
        SITL_frame = typecast(raw_bytes(5:8), 'uint32');
        
        % 16 channels of uint16 motor PWMs
        pwm_raw    = raw_bytes(9:40);
        pwm_in     = double(typecast(pwm_raw, 'uint16'))';
        
        % Check the magic value is what we expect
        if magic ~= 18458
            warning('Incorrect magic value: %d', magic);
            continue;
        end
        
        % Check if the frame is in expected order
        if SITL_frame < last_SITL_frame
            % Controller has reset, reset physics also
            state = init_function(state);
            connected = false;
            fprintf('Controller reset\n')
        elseif SITL_frame == last_SITL_frame
            % Duplicate frame, skip
            continue;
        elseif SITL_frame ~= last_SITL_frame + 1 && connected
            fprintf('Missed %i input frames\n', SITL_frame - last_SITL_frame - 1)
        end
        last_SITL_frame = SITL_frame;
        
        state.delta_t = min(1/frame_rate, max_timestep);
        physics_time_s = physics_time_s + state.delta_t;
        
        if ~connected
            connected = true;
            target_ip = packet_in.getAddress();
            target_port = packet_in.getPort();
            fprintf('Connected to AP Flight Controller at %s:%d\n', char(target_ip.getHostAddress()), target_port);
        end
        
        frame_count = frame_count + 1;
        
        % Run the custom 0.81kg physics step
        state = physics_function(pwm_in, state);
        
        % Build structure representing the JSON string to be sent
        JSON.timestamp = physics_time_s;
        JSON.imu.gyro = state.gyro;
        JSON.imu.accel_body = state.accel;
        JSON.position = state.position;
        JSON.attitude = state.attitude;
        JSON.velocity = state.velocity;
        
        % Encode JSON text and append outer newline buffers required by SITL
        json_payload = sprintf('\n%s\n', jsonencode(JSON));
        send_bytes = uint8(json_payload);
        
        % Transmit response packet back to ArduPilot's dynamic return port
        packet_out = DatagramPacket(send_bytes, length(send_bytes), target_ip, target_port);
        j_socket.send(packet_out);
        
        % Print performance statistics
        if rem(frame_count, print_frame_count) == 0
            total_time = toc(frame_time);
            frame_time = tic;
            time_ratio = (print_frame_count * state.delta_t) / total_time;
            fprintf("%0.2f fps, %0.2f%% of realtime\n", print_frame_count / total_time, time_ratio * 100)
        end
        
    catch ME
        if connected
            warning('Error in simulation loop execution: %s', ME.message);
        end
    end
end
end