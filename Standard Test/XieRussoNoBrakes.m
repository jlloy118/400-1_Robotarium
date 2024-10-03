% Clear workspace, close figures, and clear command window
clc;
clear all;
close all;

% Define platoon sizes to simulate (adjust as needed)
platoon_sizes = 6;  % Example with 6 robots
%platoon_sizes = [16,20];
% Set a fixed seed for reproducibility
rng(1);  
thetaii = -1.1;
thetajj = -2.6;
TwobNd = 0.3049079;
bsigma = 0.305907;
T = [eye(2) thetaii*eye(2) zeros(2); zeros(2) eye(2) thetajj*eye(2); zeros(2) zeros(2) eye(2)];
kappa_p = norm(T,2)*norm(inv(T),2);  % Example value, adjust based on system parameters
lambda_hat = 0.001; %solved using fsolve in another script
% Loop through different platoon sizes
for idx = 1:length(platoon_sizes)
    
    % Set the current platoon size
    N = platoon_sizes(idx);  
    
    %% Simulation Parameters %%
    ts = ARobotarium.time_step;   % Time step of the Robotarium
    num_runs = 1;                 % Number of simulation runs
    increment = 2;                % Step increment (if needed)
    spacing_const = 0.4;          % Desired spacing between robots
    desired_spacing = [spacing_const; 0];  % Spacing in x and y direction
    vconst = 0.10;                % Constant velocity for path traversal
    iterations = 4000;            % Total number of iterations

    % Initialize storage variables for results and errors
    disturbance_amplitude = 0.02;  % Amplitude of disturbance
    safety_on = false;             % Toggle for safety mechanism
    disturbance = zeros(1, N);     % Disturbance for each robot
    supremum_reference_error = zeros(iterations, 1);  % Supremum reference error
    spacing_error = zeros(iterations, N - 1);  % Spacing error between robots

    % Define control gains for formation control algorithm
    k0=1.2723;
    k1=0.6338;
    k2=0.1341;
    k0tau = 1.2723;
    k1tau = 0.6378;
    k2tau = 0.2362;
    kpsi = 0.0381; %OUR GAINS
    %   k0=1.2674;
    % k1=0.6312;
    % k2=0.133;
    % k0tau = 0.325;
    % k1tau = 0.162;
    % k2tau = 0.06;
    % kpsi = 0.1; - %ORIGINAL GAINS

    % Initialize matrices for robot states and positions
    dxi = zeros(2, N, iterations);
    dr1 = zeros(2, N, iterations);
    r1 = zeros(2, N, iterations);
    dr0 = zeros(2, N, iterations);
    r0 = zeros(2, N, iterations);
    xd = zeros(2, N, iterations);  % Desired positions
    xi = zeros(3, N, iterations);  % Robot positions

    positions = zeros(2, iterations, N);  % Store robot positions

    % Generate initial positions for the robots
    initial_positions = generate_initial_conditions(N, 'Spacing', 0.3);
    

    % Initialize the communication matrix as an NxN zero matrix
    comm_matrix = zeros(N, N);
    
    % Loop to set communication between consecutive vehicles
    for i = 1:N
        if i > 1
            comm_matrix(i, i-1) = 1;  % Communication with the vehicle behind
        end
        if i < N
            comm_matrix(i, i+1) = 1;  % Communication with the vehicle in front
        end
    end
    
    % Optionally, make the matrix symmetric if communication is bidirectional
    comm_matrix = comm_matrix + comm_matrix';


    % Initialize Robotarium with the specified number of robots
    r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_positions);
    
    % Create barrier certificates to avoid collisions
    unicycle_barrier_certificate = create_uni_barrier_certificate_with_boundary();

    %% Path Generation Variables %%
    % Boundary and path generation parameters
    boundaries = ARobotarium.boundaries;  % Environment boundaries
    W = boundaries(2) - boundaries(1);    % Width of the boundary
    H = boundaries(4) - boundaries(3);    % Height of the boundary
    robot_diam = ARobotarium.robot_diameter;  % Diameter of each robot
    radius = 0.7;  % Radius for rounded corners
    buffer = 0.2;  % Buffer to keep robots away from boundaries
    arc_radius = 0.5;  % Radius for circular arcs at corners

    % Adjust boundaries for the buffer
    x_min = -1.6 + buffer;
    x_max = 1.6 - buffer;
    y_min = -1 + buffer;
    y_max = 1 - buffer;

    % Calculate path segment lengths and times
    horizontal_length = (x_max - x_min) - 2 * arc_radius;  % Horizontal distance minus arcs
    vertical_length = (y_max - y_min) - 2 * arc_radius;    % Vertical distance minus arcs
    
    horizontal_time = round(horizontal_length / (vconst * ts));  % Time for horizontal movement
    vertical_time = round(vertical_length / (vconst * ts));      % Time for vertical movement
    arc_time = round(pi * arc_radius / (2 * vconst * ts));       % Time for circular arcs

    % Initialize path array
    xl = zeros(2, iterations);  % Store path points
    dxl = zeros(2, iterations);  % Store path derivatives
    t = 1;  % Initialize time index
    
    % Starting point: bottom middle of the boundary
    start_point = [0; y_min]; 
    xl(:, t) = start_point;  % Assign start point as first path point

    %% Path Generation Loop %%
    % Generate the path by repeating the movement along the boundaries
    while t < iterations
        % Move left along the bottom wall
        for i = 1:horizontal_time / 2
            if t >= iterations, break; end
            t = t + 1;
            xl(:, t) = xl(:, t-1) + [-vconst * ts; 0];  % Update position
            dxl(:, t) = [-vconst; 0];  % Update velocity
        end

        % Bottom-left arc (clockwise)
        center = [x_min + arc_radius; y_min + arc_radius];
        for i = 1:arc_time
            if t >= iterations, break; end
            t = t + 1;
            theta = 3 * pi / 2 - (i / arc_time) * (pi / 2);  % Arc angle
            xl(:, t) = center + arc_radius * [cos(theta); sin(theta)];
            dxl(:, t) = arc_radius * [sin(theta); -cos(theta)] * (pi / 2 / arc_time / ts);
        end

        % Move up along the left wall
        for i = 1:vertical_time
            if t >= iterations, break; end
            t = t + 1;
            xl(:, t) = xl(:, t-1) + [0; vconst * ts];  % Update position
            dxl(:, t) = [0; vconst];  % Update velocity
        end
    
        % Top-left arc (clockwise)
        center = [x_min + arc_radius; y_max - arc_radius];
        for i = 1:arc_time
            if t >= iterations, break; end
            t = t + 1;
            theta = pi - (i / arc_time) * (pi / 2);  % Arc angle
            xl(:, t) = center + arc_radius * [cos(theta); sin(theta)];
            dxl(:, t) = arc_radius * [sin(theta); -cos(theta)] * (pi / 2 / arc_time / ts);
        end
    
        % Move right along the top wall
        for i = 1:horizontal_time
            if t >= iterations, break; end
            t = t + 1;
            xl(:, t) = xl(:, t-1) + [vconst * ts; 0];  % Update position
            dxl(:, t) = [vconst; 0];  % Update velocity
        end
    
        % Top-right arc (clockwise)
        center = [x_max - arc_radius; y_max - arc_radius];
        for i = 1:arc_time
            if t >= iterations, break; end
            t = t + 1;
            theta = pi / 2 - (i / arc_time) * (pi / 2);  % Arc angle
            xl(:, t) = center + arc_radius * [cos(theta); sin(theta)];
            dxl(:, t) = arc_radius * [sin(theta); -cos(theta)] * (pi / 2 / arc_time / ts);
        end
    
        % Move down along the right wall
        for i = 1:vertical_time
            if t >= iterations, break; end
            t = t + 1;
            xl(:, t) = xl(:, t-1) + [0; -vconst * ts];  % Update position
            dxl(:, t) = [0; -vconst];  % Update velocity
        end
     % Bottom-right arc (clockwise)
        center = [x_max - arc_radius; y_min + arc_radius];
        for i = 1:arc_time
            if t >= iterations, break; end
            t = t + 1;
            theta = 0 - (i / arc_time) * (pi / 2);  % Arc angle
            xl(:, t) = center + arc_radius * [cos(theta); sin(theta)];
            dxl(:, t) = arc_radius * [sin(theta); -cos(theta)] * (pi / 2 / arc_time / ts);
        end
    
        % Continue along the bottom wall until reset
        for i = 1:horizontal_time / 2
            if t >= iterations, break; end
            t = t + 1;
            xl(:, t) = xl(:, t-1) + [-vconst * ts; 0];  % Update position
            dxl(:, t) = [-vconst; 0];  % Update velocity
        end
    end  % End of path generation loop

    %% Path Follow-Up for Remaining Iterations %%
    % If there are remaining iterations, the robots will stand still
    for i = t+1:iterations
        xl(:, i) = xl(:, t);  % Keep position constant
        dxl(:, i) = [0; 0];   % Velocity is zero
    end

    %% Reference Path Setup %%
    x_ref = xl;  % Assign the generated path to the reference path

    %% Desired Position Calculation %%
    % For each robot, calculate the desired position behind the virtual leader
    % Initialize an array to store the distance of each robot's reference from the leader at each time t
    distance_from_leader = zeros(N, iterations);
    
    for i = 1:N
        for t = 1:iterations
            desired_distance = spacing_const * i;  % Distance to maintain behind leader
            total_distance = 0;  % Accumulated distance along the reference path
    
            % Find the point on the reference path corresponding to desired_distance
            for k = t:-1:2
                total_distance = total_distance + norm(x_ref(:, k) - x_ref(:, k-1));
                if total_distance >= desired_distance
                    xd(:, i, t) = x_ref(:, k);  % Set desired position
                    break;
                end
            end
    
            % Handle edge cases where desired distance exceeds path length
            if total_distance < desired_distance
                xd(:, i, t) = x_ref(:, t);  % Use last known position if too far
            end
                % Save the distance each robot's reference is from the leader
            distance_from_leader(i, t) = total_distance;
        end
    end


    %% Final Goal Setup %%
    % Set final goal points for each robot
    final_goal_points = zeros(3, N);
    start_time = round((N + 1) * spacing_const / (vconst * ts)) + 2;  % Start time based on spacing
    
    for i = 1:N
        final_goal_points(1:2, i) = xd(:, i, start_time);  % Position at start time
        final_goal_points(3, i) = pi;  % All robots face left
    end

    %% Controller Initialization %%
    args = {'PositionError', 0.025, 'RotationError', 0.05};  % Controller tolerances
    init_checker = create_is_initialized(args{:});           % Initialization checker
    controller = create_waypoint_controller(args{:});        % Waypoint controller

    % Convert single integrator to unicycle dynamics
    si_to_uni_dyn = create_si_to_uni_dynamics_with_backwards_motion('LinearVelocityGain', 1, 'AngularVelocityLimit', pi);
    uni_barrier_cert = create_uni_barrier_certificate_with_boundary();  % Safety mechanism

    % Initialize robot poses and step the robotarium
    x = r.get_poses();
    r.step();

    %% Robot Initialization Loop %%
    while (~init_checker(x, final_goal_points))
        x = r.get_poses();
        dxu = controller(x,final_goal_points);
        dxu = uni_barrier_cert(dxu,x);  % Compute control input
        r.set_velocities(1:N, dxu);              % Set velocities for robots
        r.step();                                % Step the simulation
    end
    
    dxu = zeros(2,N);
    dxi = zeros(2,N,iterations);

    %% Control Loop %%
    % Start the control loop from the calculated start time

% Initialize the plot handles for the desired positions (xd) and actual positions (xi)
plot_handles_xd = gobjects(1, N);  % For desired positions
plot_handles_xi = gobjects(1, N);  % For actual positions

% Initialize text handles for labeling the desired and actual positions
text_handles_xd = gobjects(1, N);  % For desired position labels
text_handles_xi = gobjects(1, N);  % For actual position labels

for n = 1:N
    % Plot initial desired and actual positions
    plot_handles_xd(n) = plot(xd(1, n, start_time), xd(2, n, start_time), 'ks', 'MarkerSize', 5, 'MarkerFaceColor', 'k'); 
    plot_handles_xi(n) = plot(xi(1, n, start_time), xi(2, n, start_time), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r'); 
    
    % Add text labels for desired and actual positions
    text_handles_xd(n) = text(xd(1, n, start_time), xd(2, n, start_time), num2str(n), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'FontSize', 8, 'Color', 'k');
    text_handles_xi(n) = text(xi(1, n, start_time), xi(2, n, start_time), num2str(n), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left', 'FontSize', 8, 'Color', 'r');
end


for kk =  1:start_time-1
    xi(1:2,:,kk) = xd(:,:,kk);
end

% Control Loop
for t = start_time:iterations
    % Get robot poses for the current time step
    xi(:, :, t) = r.get_poses();
    
    % Add disturbances
    disturbance = disturbance_amplitude * exp(-0.1 * (t - start_time) * ts) * sin((t - start_time) * ts) + 5*disturbance_amplitude;
    
    %% Follower Control Implementation %%
    for n = 1:N
        
            % Initialize r1 and r0 at start time
            if t == start_time
                r1(:, n, t) = zeros(2, 1);
                r0(:, n, t) = zeros(2, 1);
            end
    
            % Initialize total distance traveled by the reference trajectory
            total_distance_ref = 0;
            error_zero = zeros(2, 1);
            
            for i = t:-1:2
                total_distance_ref = total_distance_ref + norm(x_ref(1:2, i) - x_ref(1:2, i-1));
                
                if total_distance_ref >= n * spacing_const
                    error_zero = x_ref(1:2, i) - xi(1:2, n);
                    desired_pos_ref = x_ref(1:2, i);
                    break;
                end
            end
            reftimestamp = i;

    
            % Calculate errors and update positions
            error_zero(:, n, t) = xd(:, n, t) - xi(1:2, n, t);
            dr1(:, n, t) = k2 * ((xl(1:2, t) - xi(1:2, n, t)) - (xl(1:2, t) - xd(1:2, n, t)));
            dr0(:, n, t) = k1 * ((xl(1:2, t) - xi(1:2, n, t)) - (xl(1:2, t) - xd(1:2, n, t)));
            dxi(:, n, t) = k0 * ((xl(1:2, t) - xi(1:2, n, t)) - (xl(1:2, t) - xd(1:2, n, t))); 
            
            % Neighbor coupling
            neighbours = find(comm_matrix(n,:) ~= 0);
            for j = neighbours
                dr1(:, n, t) = dr1(:, n, t) + k2tau * tanh(kpsi*((xi(1:2, j, t) - xi(1:2, n, t)) - (xd(1:2, j, t) - xd(1:2, n, t))));
                dr0(:, n, t) = dr0(:, n, t) + k1tau * tanh(kpsi*((xi(1:2, j, t) - xi(1:2, n, t)) - (xd(1:2, j, t) - xd(1:2, n, t))));
            end

                % Apply the changes immediately to r0 and r1 in the same time step
            r1(:, n, t) = r1(:,n,t-1) + dr1(:, n, t-1) * ts;
            dr0(:,n,t) = dr0(:,n,t) + r1(:,n,t);
            r0(:, n, t) = r0(:,n,t-1) + dr0(:, n, t-1) * ts;
            
            % Update dxi with the current r0 and disturbances
            dxi(:, n, t) = dxi(:, n, t) + r0(:, n, t) + disturbance + dxl(:, reftimestamp);
           
            for j = neighbours
                dxi(:, n, t) = dxi(:, n, t) + k0tau * tanh(kpsi*((xi(1:2, j, t) - xi(1:2, n, t)) - (xd(1:2, j, t) - xd(1:2, n, t))));
            end

            % Compute the velocity magnitude
            velocity_magnitude = norm(dxi(:,n,t));
            
            % Check if the magnitude exceeds the limit
            if velocity_magnitude > 0.2
                disp("Limits broken")
                
                % Scale the velocity vector to have a magnitude of 0.2
                dxi(:,n,t) = dxi(:,n,t) * (0.2 / velocity_magnitude);
            end



    
            % Update the plot handle for the desired position
            set(plot_handles_xd(n), 'XData', xd(1, n, t), 'YData', xd(2, n, t));
    
            % Update the plot handle for the actual position
            set(plot_handles_xi(n), 'XData', xi(1, n, t), 'YData', xi(2, n, t));
    
            % Update the text label handle to show the robot number at the desired position
            set(text_handles_xd(n), 'Position', [xd(1, n, t), xd(2, n, t)], 'String', num2str(n));
    
            % Update the text label handle to show the robot number at the actual position
            set(text_handles_xi(n), 'Position', [xi(1, n, t), xi(2, n, t)], 'String', num2str(n));
     
    end
    
    %% Error Calculations %%
    % Calculate supremum reference error
    reference_errors = zeros(N, 1);
    for i = 1:N
        reference_errors(i) = norm(xd(1:2, i, t) - xi(1:2, i, t));
    end
    supremum_reference_error(t) = max(reference_errors);

    % Calculate spacing error
    for i = 2:N
        current_spacing = norm(xi(1:2, i, t) - xi(1:2, i-1, t));
        spacing_error(t, i-1) = abs(current_spacing - spacing_const);
    end

    %% Update Robot Velocities and Step Simulation %%
    dxu = si_to_uni_dyn(dxi(:,:,t), xi(:,:,t));  % Convert to unicycle dynamics
    if safety_on
        dxu = uni_barrier_cert(dxu, xi(:,:,t));  % Apply barrier certificate if safety is enabled
    end
    r.set_velocities(1:N, dxu);  % Set velocities for robots
    r.step();  % Step the simulation
end


    %% Plot Supremum Reference Error %%
    figure;
    subplot(2, 1, 1);
    plot(start_time:iterations, supremum_reference_error(start_time:end,1));
    save("supremerrorxienomods","supremum_reference_error");
    title('Supremum Reference Error Over Time');
    xlabel('Iterations');
    ylabel('Supremum Error');
    xlim([start_time+1, iterations]);
    

    %% Plot Maximum Spacing Error %%
    subplot(2, 1, 2);
    max_spacing_error = max(spacing_error(start_time:end,:), [], 2);
    save("xienomodsspacing","max_spacing_error");
    plot(start_time:iterations, max_spacing_error);
    title('Maximum Spacing Error Over Time');
    xlabel('Iterations');
    ylabel('Maximum Spacing Error');
    xlim([start_time+1, iterations]);
    legend('Maximum Spacing Error');
    
    %% Save Final Errors and Plot Final Robot Movements %%
    maxspacingerrortotalxienomods = max(max_spacing_error);
    maxsupremreferrortotalxienomods = max(supremum_reference_error);
    final_poses = r.get_poses();
    r.debug();
    save(sprintf("supremum_error_platoon_size_%d_xie", N), 'maxsupremreferrortotalxienomods');
    save(sprintf("spacing_error_platoon_size_%d_xie", N), 'maxspacingerrortotalxienomods');


% Set maximum delay, disturbance-related bounds, and other parameters
tau_max = 0;
bsigma = 0.305907;  
TwobNd = 0.3049079;  
disturbance_amplitude = 0.001;  
m = 2;  
disturbance_scale = 1;  
p = 2;

% Initialize the bound array for each time step
bound = zeros(iterations, 1);

% Control loop for bound calculation
for t = start_time:iterations
    % Disturbance signal (specific form)
    disturbance = disturbance_amplitude * exp(-0.007 * (t - start_time)) * sin((t - start_time)*ts) + disturbance_amplitude;

    % First term: Exponentially decayed supremum reference error
    max_ref_error_initial = supremum_reference_error(1,1);
    max_sum_over_robots = 0;

    % Loop over all robots
    for i = 1:N
        % Compute the supremum over the time window [start_time - tau_max, start_time]
        sup_sum_rik = 0;
        for s = start_time+1 %not sure if this should be +1 or not.
            % Sum over the multiplex layers (k = 1 to m)
            sum_rik = 0;
            for k = 1:m
                % Add control inputs
                if k == 1
                    sum_rik = sum_rik + norm(r0(:, i, s), 2);  % r0 is the first layer
                elseif k == 2
                    sum_rik = sum_rik + norm(r1(:, i, s), 2);  % r1 is the second layer
                end

                % Additional term from the equation
                additional_sum = 0;
                for b = 0:(m-k)
                    factorial_ratio = factorial(m-1-b) / factorial(m-k-b);
                    additional_sum = additional_sum + factorial_ratio * disturbance_amplitude * (s)^(m-k-b);
                end
                sum_rik = sum_rik + additional_sum;
            end

            % Take the p-norm of the result
            p_norm_sum_rik = norm(sum_rik, p);
            
            % Supremum over time (though here we only compute for one time step, s = start_time)
            sup_sum_rik = max(sup_sum_rik, p_norm_sum_rik);
        end
        
        % Maximize over all robots
        max_sum_over_robots = max(max_sum_over_robots, sup_sum_rik);
    end
    control_term = max_sum_over_robots;

    % Final bound calculation
    bound(t) = kappa_p * exp(-lambda_hat * (t - start_time)) * (max_ref_error_initial + control_term); 

    % Fourth term: Disturbance-related bound (dynamic part of the disturbance)
    bound(t) = bound(t) + kappa_p * (1 / (bsigma - TwobNd)) * 2*disturbance_amplitude;
end

% Plot the bound with the reference error
figure;
hold on;
plot(start_time:iterations, supremum_reference_error(start_time:end, 1), 'b', 'LineWidth', 2);  
plot(start_time:iterations, bound(start_time:end, 1), 'r--', 'LineWidth', 2);  
title('Supremum Reference Error and Bound Over Time');
xlabel('Iterations');
ylabel('Error');
legend('Supremum Reference Error', 'Bound');
hold off;
    % Plot robot movements over time
    figure;
    for n = 1:N
        plot(start_time:iterations, vecnorm(squeeze(dxi(:,n,start_time:iterations))));
        hold on;
    end
    legend('1','2','3','4','5','6','7','8','9','10','11','12','13','14','15');
    hold off;
end  % End of simulation loop for platoon sizes

%% Final Plotting of Supremum and Max Spacing Errors for Different Platoon Sizes %%
figure;

% Plot Supremum Reference Error
subplot(2,1,1);
hold on;
for idx = 1:length(platoon_sizes)
    N = platoon_sizes(idx);
    load(sprintf("supremum_error_platoon_size_%d_xie", N));  % Load each saved error
    plot(N, maxsupremreferrortotalxienomods, 'ko', 'MarkerFaceColor', 'b');  % Plot dot
    line([N N], [0 maxsupremreferrortotalxienomods], 'LineStyle', '--', 'Color', 'b');  % Plot vertical line
end
title('Supremum Reference Error for Different Platoon Sizes');
xlabel('Platoon Size (N)');
ylabel('Supremum Error');
hold off;

% Plot Maximum Spacing Error
subplot(2,1,2);
hold on;
for idx = 1:length(platoon_sizes)
    N = platoon_sizes(idx);
    load(sprintf("spacing_error_platoon_size_%d_xie", N));  % Load each saved error
    plot(N, maxspacingerrortotalxienomods, 'ko', 'MarkerFaceColor', 'k');  % Plot dot
    line([N N], [0 maxspacingerrortotalxienomods], 'LineStyle', '--', 'Color', 'k');  % Plot vertical line
end
title('Max Spacing Error for Different Platoon Sizes');
xlabel('Platoon Size (N)');
ylabel('Max Spacing Error');
hold off;
