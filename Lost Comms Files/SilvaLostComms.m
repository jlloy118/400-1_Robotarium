clc;
clear all;
close all;
%platoon_sizes = [4,8,12,16,20];
platoon_sizes = [6];
for idx = 1:length(platoon_sizes)

    N = platoon_sizes(idx);  % Set current platoon size
    method = 3;
    mass = 0.08;
    % Simulation parameters
    ts = ARobotarium.time_step;
    num_runs = 1;
    increment = 2;
    spacing_const = 0.4;
    desired_spacing = [spacing_const; 0];
    vconst = 0.1;
    iterations = 4000;
    supremum_reference_error = zeros(iterations, 1);
    spacing_error = zeros(iterations, N - 1);



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



    % Store results
    max_superror_values = zeros(1, num_runs);
    disturbance_amplitude = 0.02;
    % disturbance_amplitude = 0;
    safety_on = false;

    right_extension_factor = 15;
    disturbance_frequency = 1;
    disturbance = zeros(1, N);

    Kp1 = 0.1188;
    Kp2 = 0.1188;
    KpZero = 0.6;
    Kv = 0.0121;
    KvZero = 0.6;
    Gp1 = 0.01;
    Gp2 = 0.01;
    GpZero = 0.2881;
    Gv = 0.01;
    GvZero = 0.342;
    kint = 0.2508;

    veq = [-vconst; 0];

    dxi = zeros(2, N);
    xi = zeros(3, N,iterations);
    integral_error = zeros(2, N);

    epsilon = 1;
    acceleration_input = zeros(2, N);

    cbarsq = 0.0076;
    alpha = 0.3;
    beta = -0.4;
    k = 0.2508;

    T = [1 0 alpha; 0 1 beta; 0 0 1];
    singvals = svd(T);
    sigma_max = max(singvals);
    sigma_min = min(singvals);
    K = sigma_max / sigma_min;

    time_data = zeros(iterations, 1);
    bound_data = zeros(iterations, 1);
    bound_dataC2 = zeros(iterations, 1);
    error_data = zeros(2,iterations, N);
    rear_error_data = zeros(2,iterations,N);
    velocity_data = zeros(2,iterations, N);
    acceleration_data = zeros(2,iterations, N);
    Hii_data = zeros(2,iterations, N);
    Hii1_data = zeros(2,iterations, N);
    Hii_minus1_data = zeros(2,iterations, N);
    H0_data = zeros(2,iterations, N);
    superror = zeros(iterations, N);
    refpositions = zeros(2,iterations, N);
    positions = zeros(2,iterations,N);
    referror = zeros(2,iterations,N);

    % Define blackout parameters
    blackout_duration = 3; % Duration of the blackout in seconds
    blackout_iterations = blackout_duration / ts;
    blackout_start_time = 3000; % Start in the middle of the simulation
    affected_robots = N-1:-4:1;  % Specify which robots will lose communication
    % Initialize variables to store last known errors
    last_front_error = zeros(2, N);  % Store the last known front error
    last_rear_error = zeros(2, N);   % Store the last known rear error
    last_front_vel_error = zeros(2,N);
    last_rear_vel_error = zeros(2,N);

    initial_positions = generate_initial_conditions(N, 'Spacing', 0.1);
    r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_positions);

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


    % Initialization logic remains the same
    args = {'PositionError', 0.025, 'RotationError', 0.05};
    init_checker = create_is_initialized(args{:});
    controller = create_waypoint_controller(args{:});

    x = r.get_poses();
    r.step();

    si_to_uni_dyn = create_si_to_uni_dynamics_with_backwards_motion('LinearVelocityGain', 1, 'AngularVelocityLimit', pi);
    uni_barrier_cert = create_uni_barrier_certificate_with_boundary();

    while (~init_checker(x, final_goal_points))
        x = r.get_poses();
        dxu = controller(x, final_goal_points);
        %dxu = uni_barrier_cert(dxu, x);
        r.set_velocities(1:N, dxu);
        r.step();
    end
    leader_index = 1;
    rear_index = N;

    dx = zeros(2, N);


    % Define the radius and center of the circular path

    center = [(boundaries(1) + boundaries(2)) / 2; (boundaries(3) + boundaries(4)) / 2];  % Center of the circle



    % Initialize variables related to initial positions and errors
    xi_initial = final_goal_points(1:2, :);
    integral_error_initial = zeros(1, N);

    x_star_initial = zeros(3, N);
    for i = 1:N
        x_star_initial(1, i) = final_goal_points(1,i);
        x_star_initial(2, i) = final_goal_points(2,i);
        x_star_initial(3, i) = final_goal_points(3,i);
    end

    for kk =  1:start_time-1
        xi(1:2,:,kk) = xd(:,:,kk);
    end

    spacing_storage = zeros(2,t,N);


   % Initialize matrices to store available positions and velocities for each robot
    % Dimensions: positions (3 x N x N), velocities (2 x N x N)
    current_poses = zeros(3, N, N);
    current_dxi = zeros(2, N, N);

    % Initialize last known positions and velocities (before blackout)
    last_known_positions = zeros(3, N, N);
    last_known_dxi = zeros(2, N, N);

    for t = start_time:iterations
        % Retrieve the current positions and velocities of all robots
        actual_positions = r.get_poses();  % Size: (3, N)
        actual_dxi = dxi;                   % Size: (2, N)

        % At each time step, update the accessible positions and velocities
        for n = 1:N
            if t == blackout_start_time - 1
                % Store last known positions and velocities before blackout
                last_known_positions(:, :, n) = current_poses(:, :, n);
                last_known_dxi(:, :, n) = current_dxi(:, :, n);
            end

            if t >= blackout_start_time && t < blackout_start_time + blackout_iterations
                % During blackout
                if ismember(n, affected_robots)
                    % Affected robots cannot receive updates from others
                    % Use last known positions and velocities for other robots
                    current_poses(:, :, n) = last_known_positions(:, :, n);
                    current_dxi(:, :, n) = last_known_dxi(:, :, n);

                    % Update own position and velocity
                    current_poses(:, n, n) = actual_positions(:, n);
                    current_dxi(:, n, n) = actual_dxi(:, n);
                else
                    % Non-affected robots
                    % Get neighbors
                    neighbors = find(comm_matrix(n, :) ~= 0);

                    % Accessible robots are neighbors not affected by the blackout
                    accessible_robots = setdiff(neighbors, affected_robots);

                    % Update positions and velocities of accessible robots
                    current_poses(:, accessible_robots, n) = actual_positions(:, accessible_robots);
                    current_dxi(:, accessible_robots, n) = actual_dxi(:, accessible_robots);

                    % For affected neighbors, use last known positions and velocities
                    affected_neighbors = intersect(neighbors, affected_robots);
                    current_poses(:, affected_neighbors, n) = last_known_positions(:, affected_neighbors, n);
                    current_dxi(:, affected_neighbors, n) = last_known_dxi(:, affected_neighbors, n);

                    % Update own position and velocity
                    current_poses(:, n, n) = actual_positions(:, n);
                    current_dxi(:, n, n) = actual_dxi(:, n);

                    % For non-neighboring robots, you can choose to keep last known values or set them to zero
                    % Here, we keep the last known values for consistency
                    non_neighbors = setdiff(1:N, neighbors);
                    current_poses(:, non_neighbors, n) = last_known_positions(:, non_neighbors, n);
                    current_dxi(:, non_neighbors, n) = last_known_dxi(:, non_neighbors, n);
                end
            else
                % Before or after blackout, normal operation
                % All robots know the current positions and velocities of all robots
                current_poses(:, :, n) = actual_positions;
                current_dxi(:, :, n) = actual_dxi;
            end
        end

        % Apply disturbances (if any)
        disturbance = disturbance_amplitude * exp(-0.1 * (t - start_time) * ts) * sin((t - start_time) * ts) + 5 * disturbance_amplitude;

        %% Follower control
        for n = 1:N
           xi(:, :, t) = current_poses(:, :, n);
            dxi = current_dxi(:, :, n);     
            if n == 1
                % The first robot (leader) just follows the reference path
                pos_ref = x_ref(1:2, t);
                front_error = [0; 0];
                front_vel_error = [0; 0];
            else
                % For other robots, find the point on the path that is spacing_const behind the robot in front
                total_distance = 0;

                for i = t-1:-1:1
                    total_distance = total_distance + norm(xi(1:2, n-1,i) - xi(1:2, n-1,i+1));
                    if total_distance >= spacing_const
                        desired_pos_front = xi(1:2, n-1,i);
                        break;
                    end
                end

                front_error = desired_pos_front - xi(1:2, n,t);
                front_vel_error = dxi(1:2, n - 1) - dxi(1:2, n);
            end

            if n ~= N
                % Initialize variables for distance calculation
                total_distance = 0;
                desired_pos_rear = xi(1:2, n,t);  % Start with the current position of the robot
                savedkk = 1;
                % Loop backward over the path traced by the current robot
                for kk = t:-1:2
                    % Sum the distances between consecutive points
                    total_distance = total_distance + norm(xi(1:2,n,kk) - xi(1:2, n,kk-1));

                    % Check if the total distance is greater than or equal to spacing_const
                    if total_distance >= spacing_const
                        desired_pos_rear = xi(1:2, n,kk);
                        break;
                    end
                    savedkk = kk;
                end


                % Calculate the rear error as the difference between the robot behind's coordinate and the desired position
                rear_error = desired_pos_rear -  xi(1:2, n+1,t);

                % Calculate the rear velocity error
                rear_vel_error = dxi(1:2,n+1) - dxi(1:2,n);
            else
                % If it's the last robot, there's no rear robot to compare against
                rear_error = [0; 0];
                rear_vel_error = [0; 0];
            end

            % Initialize total distance traveled by the reference trajectory
            total_distance_ref = 0;
            error_zero = zeros(2, 1);
            for i = t:-1:2
                total_distance_ref = total_distance_ref + norm(x_ref(1:2, i) - x_ref(1:2, i-1));

                if total_distance_ref >= n * spacing_const
                    error_zero = x_ref(1:2, i) - xi(1:2, n,t);
                    desired_pos_ref = x_ref(1:2, i);
                    break;
                end
            end

            reftimestamp = i;

            if total_distance_ref < n * spacing_const
                if n == 1
                    error_zero = x_ref(1:2, t) - xi(1:2, n,t) + [spacing_const; 0];
                else
                    error_zero = front_error;
                end
            end



            % Control logic remains the same, but with dynamically calculated target positions
            acceleration_input(:, n) = HMinusOne(HiP(Kp1, Kp2, front_error), Kv, front_vel_error) ...
                + epsilon * HPlusOne(HiP(Kp1, Kp2, rear_error), Kv, rear_vel_error) ...
                + HZero(KpZero, error_zero, KvZero, dxl(:, reftimestamp), dxi(1:2, n)) + disturbance;

            IE_dt = GMinusOne(HiP(Gp1, Gp2, front_error), Gv, front_vel_error) ...
                + epsilon * GPlusOne(HiP(Gp1, Gp2, rear_error), Gv, rear_vel_error) ...
                + GZero(GpZero, error_zero, GvZero, dxl(:, reftimestamp), dxi(1:2, n));

            integral_error(:, n) = integral_error(:, n) + IE_dt * ts;
            acceleration_input(:, n) = acceleration_input(:, n) + kint .* integral_error(:, n);

            % Store the errors and other data for analysis
            error_data(:, t, n) = front_error;
            velocity_data(:, t, n) = dxi(1:2, n);
            acceleration_data(:, t, n) = acceleration_input(:, n);
            rear_error_data(:, t, n) = rear_error;
            positions(:, t:iterations, n) = repmat(xi(1:2, n,t), 1, iterations - t + 1);
            spacing_storage(:,t,n) = max(front_error,rear_error);
            referror(:, t, n) = error_zero;
            disp(rear_error)
        end
        %% Error Calculations %%
        % Calculate supremum reference error
         reference_errors = zeros(N, 1);
        for i = 1:N
            reference_errors(i) = norm(xd(1:2, i, t) - actual_positions(1:2, i));
        end
        supremum_reference_error(t) = max(reference_errors);

        % Calculate spacing error
        for i = 2:N
            current_spacing = norm(actual_positions(1:2, i-1) - actual_positions(1:2, i));
            spacing_error(t, i-1) = abs(current_spacing - spacing_const);
        end

        for i = 1:N
            if method == 3
                dxi(1, i) = dxi(1, i) + acceleration_input(1, i)/mass * ts;
                dxi(2, i) = dxi(2, i) + acceleration_input(2, i)/mass * ts;
                % else
                %     dxi(1, i) = acceleration_input(1, i);
                %     dxi(2, i) = acceleration_input(2, i);
            end
        end


        dxu = si_to_uni_dyn(dxi, xi(:,:,t));
        if safety_on == true
            dxu = uni_barrier_cert(dxu, xi(:,:,t));
        end
        r.set_velocities(1:N, dxu);
        r.step();
    end

    final_poses = r.get_poses();

    r.debug();

    hold off;

    figure;
    subplot(2, 1, 1);
    plot(start_time:iterations, supremum_reference_error(start_time:end,1));
    save("supremerrorsilvablackout","supremum_reference_error");
    title('Supremum Reference Error Over Time');
    xlabel('Iterations');
    ylabel('Supremum Error');
    xlim([start_time+1,iterations])

    subplot(2, 1, 2);
    max_spacing_error = max(spacing_error(start_time:end,:), [], 2); % Find max error at each time step
    save("silvablackoutspacing","max_spacing_error")

    plot(start_time:iterations, max_spacing_error);
    title('Maximum Spacing Error Over Time');
    xlabel('Iterations');
    ylabel('Maximum Spacing Error');
    xlim([start_time+1, iterations]);
    legend('Maximum Spacing Error');
    maxspacingerrortotalsilvalostcomms = max(max_spacing_error);
    maxsupremreferrortotalsilvalostcomms = max(supremum_reference_error);

    save(sprintf("supremum_error_platoon_size_%d_silva", N), 'maxsupremreferrortotalsilvalostcomms');
    save(sprintf("spacing_error_platoon_size_%d_silva", N), 'maxspacingerrortotalsilvalostcomms');
    hold off;
end
% After the for-loop ends
figure;


%% Plot Supremum Reference Error %%
subplot(2,1,1);
hold on;
for idx = 1:length(platoon_sizes)
    N = platoon_sizes(idx);
    load(sprintf("supremum_error_platoon_size_%d_silva", N));  % Load each saved error

    % Plot the dot
    plot(N, maxsupremreferrortotalsilvalostcomms, 'ko', 'MarkerFaceColor', 'b');  % Black filled dot

    % Plot the vertical dotted line
    line([N N], [0 maxsupremreferrortotalsilvalostcomms], 'LineStyle', '--', 'Color', 'b');  % Vertical line
end
title('Supremum Reference Error for Different Platoon Sizes');
xlabel('Platoon Size (N)');
ylabel('Supremum Error');
legend('Supremum Error');
hold off;

%% Plot Max Spacing Error %%
subplot(2,1,2);
hold on;
for idx = 1:length(platoon_sizes)
    N = platoon_sizes(idx);
    load(sprintf("spacing_error_platoon_size_%d_silva", N));  % Load each saved error

    % Plot the dot
    plot(N, maxspacingerrortotalsilvalostcomms, 'ko', 'MarkerFaceColor', 'k');  % Black filled dot

    % Plot the vertical dotted line
    line([N N], [0 maxspacingerrortotalsilvalostcomms], 'LineStyle', '--', 'Color', 'k');  % Vertical line
end
title('Max Spacing Error for Different Platoon Sizes');
xlabel('Platoon Size (N)');
ylabel('Max Spacing Error');
legend('Max Spacing Error');
hold off;


%% Functions To Save Space
function [hip] = HiP(GainOne, GainTwo, x)
hip = GainOne * tanh(GainTwo * x);
end

function [hminone] = HMinusOne(hip, Kv, front_vel_err)
hminone = hip + Kv * front_vel_err;
end

function [hplusone] = HPlusOne(hip, Kv, rear_vel_err)
hplusone = hip + Kv * rear_vel_err;
end

function [hzero] = HZero(KpZero, error_zero, KvZero, velzero, vel)
hzero = KpZero * (error_zero) + KvZero * (velzero - vel);
end

function [gminone] = GMinusOne(gip, Gv, front_vel_err)
gminone = gip + Gv * front_vel_err;
end

function [gplusone] = GPlusOne(gip, Gv, rear_vel_err)
gplusone = gip + Gv * rear_vel_err;
end

function [gzero] = GZero(GpZero, error_zero, GvZero, velzero, vel)
gzero = GpZero * (error_zero) + GvZero * (velzero - vel);
end