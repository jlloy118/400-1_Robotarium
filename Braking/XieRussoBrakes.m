clc;
clear workspace;
close all;

% Define platoon sizes to simulate (adjust as needed)
 platoon_sizes = 6;  % Example with 6 robots
%platoon_sizes = [4,8,12,16,20];
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
    
N = platoon_sizes(idx);  % Set current platoon size

% Simulation parameters
ts = ARobotarium.time_step;

spacing_const = 0.4;
vconst = 0.1;
iterations = 4000;

% Store results
disturbance_amplitude = 0.02;
safety_on = false;


disturbance = zeros(1, N);
supremum_reference_error = zeros(iterations, 1);
spacing_error = zeros(iterations, N - 1);



% These are gains for our formation control algorithm
k0=1.2723;
k1=0.6338;
k2=0.1341;
k0tau = 1.2723;
k1tau = 0.6378;
k2tau = 0.2362;
kpsi = 0.0381;

dxi = zeros(2, N, iterations);
dr1 = zeros(2, N, iterations);
r1 = zeros(2, N, iterations);
dr0 = zeros(2, N, iterations);
r0 = zeros(2, N, iterations);
xd = zeros(2, N, iterations);
xi = zeros(3, N, iterations);

positions = zeros(2,iterations,N);

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


initial_positions = generate_initial_conditions(N, 'Spacing', 0.1);
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_positions);

unicycle_barrier_certificate = create_uni_barrier_certificate_with_boundary();

%% Path Generation Variables %%
% Boundary and path generation parameters
boundaries = ARobotarium.boundaries;  % Environment boundaries
W = boundaries(2) - boundaries(1);    % Width of the boundary
H = boundaries(4) - boundaries(3);    % Height of the boundary
robot_diam = ARobotarium.robot_diameter;  % Diameter of each robot
buffer = 0.2;  % Buffer to keep robots away from boundaries
arc_radius = 0.5;  % Radius for circular arcs at corners

% Adjust boundaries for the buffer
x_min = -1.6 + buffer;
x_max = 1.6 - buffer;
y_min = -1 + buffer;
y_max = 1 - buffer;


% Adjusted horizontal and vertical lengths to account for arcs
horizontal_length = (x_max - x_min) - 2 * arc_radius;
vertical_length = (y_max - y_min) - 2 * arc_radius;

% Parameters for deceleration and stopping
deceleration_time = round(3 / ts); % Time steps for deceleration (make it a few seconds long)
stop_time = round(5 / ts);         % Time steps to stop (make the stop a few seconds long)
brake_vconst = vconst / 5;         % Reduced velocity during braking

% Calculate the distance traveled during deceleration
deceleration_distance = (vconst + brake_vconst) / 2 * (deceleration_time * ts);

% Recalculate horizontal travel time after deceleration and stop
remaining_horizontal_length = horizontal_length - deceleration_distance;
remaining_horizontal_time = round(remaining_horizontal_length / (vconst * ts));

% Time for vertical segments and arcs
vertical_time = round(vertical_length / (vconst * ts));
horizontal_time = round(horizontal_length/(vconst*ts));
arc_time = round(pi * arc_radius / (2 * vconst * ts));
t = 1;

% Starting point: Bottom middle of the boundary
start_point = [0; y_min];
xl(:, t) = start_point;

   while t < iterations
        % Move left along the bottom wall
        for i = 1:horizontal_time / 2
            t = t + 1;
            xl(:, t) = xl(:, t-1) + [-vconst * ts; 0];  % Update position
            dxl(:, t) = [-vconst; 0];  % Update velocity
        end

        % Bottom-left arc (clockwise)
        center = [x_min + arc_radius; y_min + arc_radius];
        for i = 1:arc_time
            t = t + 1;
            theta = 3 * pi / 2 - (i / arc_time) * (pi / 2);  % Arc angle
            xl(:, t) = center + arc_radius * [cos(theta); sin(theta)];
            dxl(:, t) = arc_radius * [sin(theta); -cos(theta)] * (pi / 2 / arc_time / ts);
        end

        % Move up along the left wall
        for i = 1:vertical_time
            t = t + 1;
            xl(:, t) = xl(:, t-1) + [0; vconst * ts];  % Update position
            dxl(:, t) = [0; vconst];  % Update velocity
        end
    
        % Top-left arc (clockwise)
        center = [x_min + arc_radius; y_max - arc_radius];
        for i = 1:arc_time
            t = t + 1;
            theta = pi - (i / arc_time) * (pi / 2);  % Arc angle
            xl(:, t) = center + arc_radius * [cos(theta); sin(theta)];
            dxl(:, t) = arc_radius * [sin(theta); -cos(theta)] * (pi / 2 / arc_time / ts);
        end

        % 5. Move right along the top wall with braking and stop
        % Decelerate
        for i = 1:deceleration_time
            t = t + 1;
            current_speed = vconst - ((vconst - brake_vconst) * (i / deceleration_time));  % Gradual deceleration
            xl(:, t) = xl(:, t-1) + [current_speed * ts; 0];
            dxl(:, t) = [current_speed; 0];
        end
    
        % Stop
        for i = 1:stop_time
            t = t + 1;
            xl(:, t) = xl(:, t-1);  % Stay at the same position
            dxl(:, t) = [0; 0];     % No velocity while stopped
        end
    
        % Continue moving right after the stop at constant speed
        for i = 1:(remaining_horizontal_time)
            t = t + 1;
            xl(:, t) = xl(:, t-1) + [vconst * ts; 0];
            dxl(:, t) = [vconst; 0];
        end

        % Top-right arc (clockwise)
        center = [x_max - arc_radius; y_max - arc_radius];
        for i = 1:arc_time
            t = t + 1;
            theta = pi / 2 - (i / arc_time) * (pi / 2);  % Arc angle
            xl(:, t) = center + arc_radius * [cos(theta); sin(theta)];
            dxl(:, t) = arc_radius * [sin(theta); -cos(theta)] * (pi / 2 / arc_time / ts);
        end
    
        % Move down along the right wall
        for i = 1:vertical_time
            t = t + 1;
            xl(:, t) = xl(:, t-1) + [0; -vconst * ts];  % Update position
            dxl(:, t) = [0; -vconst];  % Update velocity
        end
    
        % Bottom-right arc (clockwise)
        center = [x_max - arc_radius; y_min + arc_radius];
        for i = 1:arc_time
            t = t + 1;
            theta = 0 - (i / arc_time) * (pi / 2);  % Arc angle
            xl(:, t) = center + arc_radius * [cos(theta); sin(theta)];
            dxl(:, t) = arc_radius * [sin(theta); -cos(theta)] * (pi / 2 / arc_time / ts);
        end
    
        % Continue along the bottom wall until reset
        for i = 1:horizontal_time / 2
            t = t + 1;
            xl(:, t) = xl(:, t-1) + [-vconst * ts; 0];  % Update position
            dxl(:, t) = [-vconst; 0];  % Update velocity
        end
    end  % End of path generation loop


  % Reference path for the leader
    x_ref = xl;

   distance_from_leader = zeros(N, iterations);
 % Compute cumulative distance along the leader's path
cumulative_distance = zeros(1, iterations);
for t = 2:iterations
    cumulative_distance(t) = cumulative_distance(t-1) + norm(x_ref(:, t) - x_ref(:, t-1));
end
for i = 1:N
    desired_distance = spacing_const * i;  % Distance to maintain behind leader
    for t = 1:iterations
        desired_cumulative_distance = cumulative_distance(t) - desired_distance;
        
        if desired_cumulative_distance <= 0
            % If desired cumulative distance is less than zero, use leader's starting position
            xd(:, i, t) = x_ref(:, 1);
            distance_from_leader(i, t) = cumulative_distance(t);
        else
            % Find the index k where cumulative_distance(k) >= desired_cumulative_distance
            k = find(cumulative_distance >= desired_cumulative_distance, 1, 'first');
            if isempty(k)
                % If desired cumulative distance exceeds path length, use last position
                xd(:, i, t) = x_ref(:, end);
                distance_from_leader(i, t) = cumulative_distance(end) - cumulative_distance(end);
            else
                xd(:, i, t) = x_ref(:, k);
                distance_from_leader(i, t) = cumulative_distance(t) - cumulative_distance(k);
            end
        end
    end
end


    %% Final Goal Setup %%
    % Set final goal points for each robot
    final_goal_points = zeros(3, N);
    start_time = round((N + 1) * spacing_const / (vconst * ts)) +stop_time;  % Start time based on spacing
    
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
        %dxu = uni_barrier_cert(dxu,x);  % Compute control input
        r.set_velocities(1:N, dxu);              
        r.step();                                
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
        spacing_error(t, i) = abs(current_spacing - spacing_const);
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
    save("supremerrorxiebrakes","supremum_reference_error");
    title('Supremum Reference Error Over Time');
    xlabel('Iterations');
    ylabel('Supremum Error');
    xlim([start_time+1, iterations]);
    

    %% Plot Maximum Spacing Error %%
    subplot(2, 1, 2);
    max_spacing_error = max(spacing_error(start_time:end,:), [], 2);
    save("xiebrakessspacing","max_spacing_error");
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
