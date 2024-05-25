clc;
clear all;
close all;

method = 1; % 1 or 3. 1 is the acceleration controller simply modeled as a velocity controller. 3 is an acceleration controller integrated to give velocity commands.

% Simulation parameters
ts = ARobotarium.time_step; % Robotarium Default, replace with ARobotarium.time_step if available
initial_N = 5; % Initial number of robots
num_runs = 6; % Number of times to increase the number of robots
increment = 2; % Increment in the number of robots
right_extension_factor = 15;

iterations = 100; % Number of iterations for each simulation
% Store results
max_superror_values = zeros(1, num_runs);
robot_counts = initial_N + (0:num_runs-1) * increment;
disturbance_amplitude = rand(1); % Amplitude of the sinusoidal disturbance
disturbance_amplitude = disturbance_amplitude;

for run = 1:num_runs
    N = initial_N + (run - 1) * increment; % Number of robots for this run
    % Disturbance parameters
    disturbance_frequency = 1; % Frequency of the sinusoidal disturbance
    
    % Initialize disturbance
    disturbance = zeros(1, N);

    % Initial Control GAINS
    Kp1 = 0.1188; % Proportional gain for position
    Kp2 = 0.1188; % Proportional gain for position shaping
    KpZero = 0.6;
    Kv = 0.0121; % Velocity gain
    KvZero = 0.6;
    Gp1 = 0.01; % Proportional gain for integral action
    Gp2 = 0.01; % Proportional gain for integral action shaping
    GpZero = 0.2881;
    Gv = 0.01; % Velocity gain for integral action
    GvZero = 0.342;
    kint = 0.2508;
    veq = 0.5;

    % Initialize control input, state matrices, and integral state
    dxi = zeros(2, N); % Control inputs
    xi = zeros(3, N); % States [x; y; theta]
    integral_error = zeros(1, N); % Integral term for disturbance rejection
    desired_spacing = 1;
    epsilon = 1;
    acceleration_input = zeros(1, N); % Acceleration input for control

    %% Bound variables
    cbarsq = 0.0076;
    alpha = 0.3;
    beta = -0.4;
    k = 0.2508;

    T = [1 0 alpha; 0 1 beta; 0 0 1];
    singvals = svd(T);
    sigma_max = max(singvals);
    sigma_min = min(singvals);
    K = sigma_max / sigma_min;

    % Initialize data storage for plotting
    time_data = zeros(iterations, 1);
    bound_data = zeros(iterations, 1);
    bound_dataC2 = zeros(iterations, 1);
    error_data = zeros(iterations, N); % For front and back errors
    velocity_data = zeros(iterations, N); % To store velocity data
    acceleration_data = zeros(iterations, N); % To store acceleration data
    Hii_data = zeros(iterations, N);
    Hii1_data = zeros(iterations, N);
    Hii_minus1_data = zeros(iterations, N);
    H0_data = zeros(iterations, N);
    superror = zeros(iterations, N);
    refpositions = zeros(iterations, N);

    % Initialize Robotarium instance
    initial_positions = generate_initial_conditions(N, 'Spacing', 0.1);
    r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_positions);

    % Create a barrier certificate for use with the above parameters 
    unicycle_barrier_certificate = create_uni_barrier_certificate_with_boundary();

    boundaries = ARobotarium.boundaries;
    W = boundaries(2) - boundaries(1); % Width of the arena
    H = boundaries(4) - boundaries(3); % Height of the arena

    % Set all robots at a fixed height, midway vertically
    fixed_y = (boundaries(3) + boundaries(4)) / 2;
    
    % Compute the horizontal spacing between robots
    spacing = desired_spacing + disturbance_amplitude; % Desired spacing plus disturbance amplitude
    
    % Generate final goal points
    final_goal_points = zeros(3, N);
  
    for i = 1:N
        final_goal_points(1,i) = final_goal_points(1,1) + (i-1) * desired_spacing + disturbance_amplitude;
        final_goal_points(2, i) = fixed_y; % y-coordinate
        final_goal_points(3, i) = pi; 
    end

    args = {'PositionError', 0.025, 'RotationError', 0.05};
    init_checker = create_is_initialized(args{:});
    controller = create_waypoint_controller(args{:});

    % Get initial location data for while loop condition
    x = r.get_poses();
    r.step();

    while (~init_checker(x, final_goal_points))
        x = r.get_poses();
        dxu = controller(x, final_goal_points) * 10; %% Greatly speeds up the setup time.
        r.set_velocities(1:N, dxu);
        r.step();
    end
    disp(x)
    
    % Tools for dynamics transformation and ensuring safety
    si_to_uni_dyn = create_si_to_uni_dynamics('LinearVelocityGain', 1, 'AngularVelocityLimit', pi / 2);
    uni_barrier_cert = create_uni_barrier_certificate();

    % Leader and rear identification
    leader_index = 1; % Assuming first vehicle is the leader
    rear_index = N; % Assuming last vehicle is in the rear

    % For analytics
    dx = zeros(2, N); % Initialize velocities

    % Initialize leader's trajectory
    xl = final_goal_points(:, 1);

    % Define the dynamic path
    dxl = zeros(2, iterations);
    % Straight line
    dxl(:, 1:1000) = [-veq * ones(1, 1000); zeros(1, 1000)];
    % Continuing straight
    dxl(:, 1001:iterations) = [-veq * ones(1, iterations - 1000); zeros(1, iterations - 1000)];

    % Update the leader's position based on the velocity
    for t = 1:iterations - 1
        xl(1:2, t + 1) = xl(1:2, t) + dxl(:, t) * ts;
    end

    % Assuming the leader starts with a velocity in the x direction
    if norm(dxl(:, 1)) > 0
        direction = dxl(:, 1) / norm(dxl(:, 1));
    else
        direction = [1; 0]; % Default direction if the leader starts at rest
    end

    left_rgbs = [0; 255; 0] * ones(1, N);
    right_rgbs = [255; 0; 0] * ones(1, N);

    % Set left and right LED colors
    r.set_left_leds(1:N, left_rgbs);
    r.set_right_leds(1:N, right_rgbs);

    % Virtual leader's reference trajectory
    x_ref = xl;

    % Store initial positions
    xi_initial = final_goal_points(1, :);
    integral_error_initial = zeros(1, N);

    x_star_initial = zeros(3, N);
    for i = 1:N
        x_star_initial(1, i) = x_ref(1, 1) + (i - 1) * desired_spacing;
        x_star_initial(2, i) = x_ref(2, 1);
        x_star_initial(3, i) = x_ref(3, 1);
    end

    % Create the VideoWriter object
    v = VideoWriter(sprintf('Method %d Uncapped.avi', method));
    v.FrameRate = 30;
    open(v);

    % Initialize text handles
    text_handles = gobjects(1, N);
    rear_text = gobjects(1, N);

    %% START MAIN SIM

    %% Control Loop
    for t = 1:iterations
        % Update gains from the base workspace
        Kp1 = evalin('base', 'Kp1');
        Kp2 = evalin('base', 'Kp2');
        Kv = evalin('base', 'Kv');

        xi = r.get_poses();
        disturbance = disturbance_amplitude * exp(-0.007 * t) * sin(disturbance_frequency * t) * ones(1, N) + disturbance_amplitude; % Decaying sinusoidal disturbance,-0.007 means disturbance gone by t ~= 1000

        % Calculate cumulative distances from the virtual leader for each robot at time t
        cumulative_distance = desired_spacing * (N - 1);

        %% Follower control
        for n = 1:N
            q0 = x_ref(1, t); % Reference position of the virtual leader

            if n == 1
                % For the first robot (leader), calculate error with respect to the virtual leader
                pos_ref = x_ref(1, t); % Reference position for the first robot (virtual leader position)
                front_error = pos_ref - xi(1, n) + desired_spacing; % Error with respect to virtual leader
                front_vel_error = veq - abs(dxi(1, n));
            else
                % For other robots, calculate error with respect to the robot in front
                pos_ref = xi(1, n - 1); % Reference position for the n-th robot
                front_error = pos_ref - xi(1, n) + desired_spacing; % Error with respect to the robot in front
                front_vel_error = dxi(1, n - 1) - dxi(1, n);
            end

            if n ~= N
                % For robots that are not the last, calculate rear error
                rear_error = xi(1, n + 1) - xi(1, n) - desired_spacing;
                rear_vel_error = dxi(1, n) - dxi(1, n + 1);
            else
                rear_error = 0;
                rear_vel_error = 0;
            end
            velzero = veq;

            % Calculate cumulative desired spacing from the virtual leader to the i-th robot
            delta_i0 = desired_spacing * (n - 1);
            delta_i1 = desired_spacing * n;
            %% Control Input %%
            acceleration_input(n) = HMinusOne(HiP(Kp1, Kp2, front_error), Kv, front_vel_error) ...
                + epsilon * HPlusOne(HiP(Kp1, Kp2, rear_error), Kv, rear_vel_error) ...
                + HZero(KpZero, q0, xi(1, n), delta_i0, KvZero, velzero, dxi(1, n));

            %% Add Integral Gain Terms
            IE_dt = GMinusOne(HiP(Gp1, Gp2, front_error), Gv, front_vel_error) ...
                + epsilon * GPlusOne(HiP(Gp1, Gp2, rear_error), Gv, rear_vel_error) ...
                + GZero(GpZero, q0, xi(1, n), delta_i0, GvZero, velzero, dxi(1, n));

            % Update the integral error
            integral_error(n) = integral_error(n) + IE_dt * ts;

            if t == 1
                integral_error_initial = integral_error;
            end

            % Add integral_error to control input
            acceleration_input(n) = acceleration_input(n) + kint * integral_error(n);
            acceleration_input(n) = acceleration_input(n) + disturbance(n);
            max_accel = ARobotarium.max_linear_velocity * 100;

            if abs(acceleration_input(n)) > max_accel
                acceleration_input(n) = sign(acceleration_input(n)) * max_accel;
            end

            % Store front and rear errors for plotting
            error_data(t, n) = front_error; % 0 is good
            velocity_data(t, n) = dxi(1, n); % Store the velocity
            acceleration_data(t, n) = acceleration_input(n); % Store the acceleration input
            rear_error_data(t, n) = rear_error; % Store the rear error
            refpositions(t, n) = x_ref(1, t) + delta_i1;
        end

        %% EQ 22, Bound C1
        sup_xi0 = max(vecnorm(xi_initial(1, :)' - x_star_initial(1, :)', 2, 2));
        sup_disturbance = max(vecnorm(disturbance_amplitude + disturbance, Inf, 2));
        bound = K * exp(-cbarsq * t * ts) * sup_xi0 + K * ((1 - exp(-cbarsq * t * ts)) / cbarsq) * sup_disturbance;

        %% EQ 19, Bound of this controller
        scale = K * exp(-cbarsq * t * ts);
        sup_disturbanceC2 = max(vecnorm(disturbance, Inf, 2));
        sup_lemmadist = max(vecnorm(integral_error_initial + k^-1 * disturbance_amplitude, 2, 2));
        boundC2 = scale * sup_xi0 + scale * sup_lemmadist + K * ((1 - exp(-cbarsq * t * ts)) / cbarsq) * sup_disturbanceC2;

        actual_sup = max(vecnorm((xi(1, :)' - refpositions(t, :)'), 2, 2));
        disp(actual_sup)
        % Store the bound for plotting
        time_data(t) = t * ts;
        bound_data(t) = bound;
        bound_dataC2(t) = boundC2;
        superror(t) = actual_sup;

        %% Integrate acceleration to get velocity
        for i = 1:N
            if method == 3
                dxi(1, i) = dxi(1, i) + acceleration_input(i) * ts;
            else
                dxi(1, i) = acceleration_input(i);
            end
        end

        %% Ensure velocities are zero in the y-direction
        dxi(2, :) = 0;

        %% Cap Velocities to 12 cm/s
        max_velocity = ARobotarium.max_linear_velocity * 50; % Maximum velocity in m/s (20 cm/s)
        for i = 1:N
            if abs(dxi(1, i)) > max_velocity
                dxi(1, i) = sign(dxi(1, i)) * max_velocity;
            end
        end

        %% Apply transformations and safety measures
        dxu = si_to_uni_dyn(dxi, xi);
        % dxu = uni_barrier_cert(dxu, xi); % Ensure robots remain safe - disable for large scale tests as we need to get outside wall
        dxu(2, :) = 0; % Ensure no y velocities after transformation to unicycle
        r.set_velocities(1:N, dxu);
        r.step();

        % Capture frame for the video
        x_min = xi(1, 1) - (right_extension_factor / 2) * desired_spacing; % Adjust left boundary
        x_max = xi(1, 1) + right_extension_factor * desired_spacing; % Adjust right boundary
        y_min = boundaries(3);
        y_max = boundaries(4);
        axis([x_min x_max y_min y_max]);

        % Clear old text
        if t > 1
            delete(text_handles);
            delete(rear_text);
        end

        % Display robot statistics above and below each robot
        % Define the horizontal offset for the text to avoid overlap
        horizontal_offset = 1;

        % Display robot statistics above each robot with horizontal offset
        for n = 1:N
            text_handles(n) = text(xi(1, n) + (n - 1) * horizontal_offset, xi(2, n) + 2, ...
                sprintf('Robot %d\nError: %.2f\nVelocity: %.2f', n, error_data(t, n), velocity_data(t, n)), ...
                'HorizontalAlignment', 'center', 'FontSize', 5, 'FontWeight', 'bold', 'Color', 'k');
            rear_text(n) = text(xi(1, n) + (n - 1) * horizontal_offset * 1.2, xi(2, n) - 2, ...
                sprintf('Rear Error: %.2f', rear_error_data(t, n)), ...
                'HorizontalAlignment', 'center', 'FontSize', 5, 'FontWeight', 'bold', 'Color', 'k');
        end

        % Plot reference positions
        % for n = 1:N
        % q0 = x_ref(1, t) + desired_spacing * (n - 1); % Corrected q0 calculation
        % plot(q0, fixed_y, 'ro', 'MarkerSize', 5, 'LineWidth', 1); % Plot the reference position as a red dot
        % text(q0, fixed_y - 0.1, sprintf('%d', n), 'Color', 'r', 'FontSize', 4, 'HorizontalAlignment', 'center'); % Add number below the dot
        % end

        % Capture frame and resize it
        frame = getframe(gcf);
        writeVideo(v, frame);

        % Debugging print statements
        if mod(t, 100) == 0
            fprintf('Time: %d\n', t);
            for n = 1:N
                fprintf('Robot %d: Position = (%.2f, %.2f), Velocity = (%.2f, %.2f), Acceleration = %.2f\n', n, xi(1, n), xi(2, n), dxi(1, n), dxi(2, n), acceleration_input(n));
            end
        end

        % Store the velocities for analytics
        dx(:, :) = dxu;

        for i = 1:N
            % h_{i,i-1}
            if i == 1
                Hii_minus1_data(t, i) = HMinusOne(HiP(Kp1, Kp2, x_ref(1, t) - xi(1, i) + desired_spacing), Kv, veq - dxi(1, i));
            else
                Hii_minus1_data(t, i) = HMinusOne(HiP(Kp1, Kp2, xi(1, i - 1) - xi(1, i) + desired_spacing), Kv, dxi(1, i - 1) - dxi(1, i));
            end

            % h_{i,i+1}
            if i < N
                Hii1_data(t, i) = HPlusOne(HiP(Kp1, Kp2, xi(1, i + 1) - xi(1, i) - desired_spacing), Kv, dxi(1, i + 1) - dxi(1, i));
            else
                Hii1_data(t, i) = NaN;
            end

            % h_{0,i}
            if i == 1
                H0_data(t, i) = HZero(KpZero, x_ref(1, t), xi(1, i), 0, KvZero, veq, dxi(1, i));
            else
                H0_data(t, i) = HZero(KpZero, xi(1, i - 1), xi(1, i), desired_spacing, KvZero, dxi(1, i - 1), dxi(1, i));
            end
        end
    end

    %% END SIMULATION

    % Store final robot poses for analytics
    final_poses = r.get_poses();

    r.debug(); % End of experiment

    % Close the VideoWriter object
    close(v);

    % Plot the collected data and calculate the coupling functions H
    figure;
    for i = 1:N
        subplot(N, 1, i);
        plot(time_data, error_data(:, i));
        title(['Error for Robot ' num2str(i)]);
        xlabel('Time (s)');
        ylabel('Error (m)');
    end

    figure;
    for i = 1:N
        subplot(N, 1, i);
        plot(time_data, velocity_data(:, i));
        title(['Velocity for Robot ' num2str(i)]);
        xlabel('Time (s)');
        ylabel('Velocity (m/s)');
    end

    figure;
    for i = 1:N
        subplot(N, 1, i);
        plot(time_data, acceleration_data(:, i));
        title(['Acceleration Input for Robot ' num2str(i)]);
        xlabel('Time (s)');
        ylabel('Acceleration (m/s^2)');
    end

    %Sup Plot
    figure;
    hold on;
    plot(time_data, superror(:, 1), 'b', 'LineWidth', 2, 'DisplayName', 'C2');
    plot(time_data, bound_dataC2, 'r--', 'LineWidth', 2, 'DisplayName', 'Bound (19)');
    title('Bounds and Actual Values Over Time');
    xlabel('Time [s]');
    ylabel('sup_i |x_i(t) - x_i^*(t)|_2');
    legend;
    hold off;

    figure;
    for i = 1:N
        subplot(N, 1, i);
        plot(time_data, Hii_minus1_data(:, i));
        title(['Coupling Function h_{i,i-1} for Robot ' num2str(i)]);
        xlabel('Time (s)');
        ylabel('h_{i,i-1}');
    end

    figure;
    for i = 1:N - 1
        subplot(N - 1, 1, i);
        plot(time_data, Hii1_data(:, i));
        title(['Coupling Function h_{i,i+1} for Robot ' num2str(i)]);
        xlabel('Time (s)');
        ylabel('h_{i,i+1}');
    end

    figure;
    for i = 1:N
        subplot(N, 1, i);
        plot(time_data, H0_data(:, i));
        title(['Coupling Function h_{0} for Robot ' num2str(i)]);
        xlabel('Time (s)');
        ylabel('h_{0}');
    end

    % Store the maximum value of superror for this run
    max_superror_values(run) = max(superror(:,1));
end

% Plot the maximum superror values against the number of robots
figure;
plot(robot_counts, max_superror_values, 'o-', 'LineWidth', 2);
title('Maximum sup_i |x_i(t) - x_i^*(t)|_2 vs. Number of Robots');
xlabel('Number of Robots');
ylabel('Maximum sup_i |x_i(t) - x_i^*(t)|_2');
xlim([initial_N,N])
ylim([0.5,1.5])
grid on;

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

function [hzero] = HZero(KpZero, poszero, pos, des_space_zero, KvZero, velzero, vel)
    hzero = KpZero * (poszero - pos + des_space_zero) + KvZero * (velzero - vel); % Change to a + sign since its a vector value now
end

function [gminone] = GMinusOne(gip, Gv, front_vel_err)
    gminone = gip + Gv * front_vel_err;
end

function [gplusone] = GPlusOne(gip, Gv, rear_vel_err)
    gplusone = gip + Gv * rear_vel_err;
end

function [gzero] = GZero(GpZero, poszero, pos, des_space_zero, GvZero, velzero, vel)
    gzero = GpZero * (poszero - pos + des_space_zero) + GvZero * (velzero - vel); % See above comment
end
