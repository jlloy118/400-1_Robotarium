clc;
clear all;
close all;
%platoon_sizes = [4,8,12,16,20];
platoon_sizes = [6];
for idx = 1:length(platoon_sizes)
    
    N = platoon_sizes(idx);  % Set current platoon size
method = 3;

% Simulation parameters
ts = ARobotarium.time_step;
m = 0.08;
spacing_const = 0.4;
vconst = 0.1;
iterations = 4000;
supremum_reference_error = zeros(iterations, 1);
spacing_error = zeros(iterations, N - 1);


% Store results
%disturbance_amplitude = 0.1/randi(10);
 disturbance_amplitude = 0.02;
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
    xi = zeros(3, N, iterations);
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

    initial_positions = generate_initial_conditions(N, 'Spacing', 0.1);
    r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_positions);

    unicycle_barrier_certificate = create_uni_barrier_certificate_with_boundary();

    % Setup Boundary Parameters
    boundaries = ARobotarium.boundaries;
    W = boundaries(2) - boundaries(1);  % Width of the boundary
    H = boundaries(4) - boundaries(3);  % Height of the boundary
    robot_diam = ARobotarium.robot_diameter;  % Diameter of the robot

    % Path Generation Parameters
    radius = 0.3;  % Radius for rounded corners

% Initialize time index and path arrays
t = 1;
xl = zeros(2, iterations);
dxl = zeros(2, iterations);
xd = zeros(2, N, iterations);
buffer = 0.2;  % Buffer to keep the robots away from the boundaries
arc_radius = 0.5;  % Radius for circular turns

% Boundary dimensions
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

% Starting point: Bottom middle of the boundary
start_point = [0; y_min];
xl(:, t) = start_point;

% Repeat the entire path generation sequence until the end of iterations
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

        % 5. Move right along the top wall with braking and stop
        % Decelerate
        for i = 1:deceleration_time
            if t >= iterations, break; end
            t = t + 1;
            current_speed = vconst - ((vconst - brake_vconst) * (i / deceleration_time));  % Gradual deceleration
            xl(:, t) = xl(:, t-1) + [current_speed * ts; 0];
            dxl(:, t) = [current_speed; 0];
        end
    
        % Stop
        for i = 1:stop_time
            if t >= iterations, break; end
            t = t + 1;
            xl(:, t) = xl(:, t-1);  % Stay at the same position
            dxl(:, t) = [0; 0];     % No velocity while stopped
        end
    
        % Continue moving right after the stop at constant speed
        for i = 1:(remaining_horizontal_time)
            if t >= iterations, break; end
            t = t + 1;
            xl(:, t) = xl(:, t-1) + [vconst * ts; 0];
            dxl(:, t) = [vconst; 0];
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
    start_time = round((N + 1) * spacing_const / (vconst * ts)) + stop_time;  % Start time based on spacing
    
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


    % Control Loop
for t = start_time:iterations
    xi(:, :, t) = r.get_poses();
    disturbance = disturbance_amplitude * exp(-0.1 * (t - start_time) * ts) * sin((t - start_time) * ts) + 5*disturbance_amplitude;

    %% Follower control
    for n = 1:N
            % dxi(1, n) = dxi(1, n) + disturbance;
            % 
            % dxi(2, n) = dxi(2, n) + disturbance;
        
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
        reference_errors(i) = norm(xd(1:2, i, t) - xi(1:2, i, t));
    end
    supremum_reference_error(t) = max(reference_errors);

    % Calculate spacing error
    for i = 2:N
        current_spacing = norm(xi(1:2, i, t) - xi(1:2, i-1, t));
        spacing_error(t, i) = abs(current_spacing - spacing_const);
    end

    for i = 1:N
        if method == 3
            dxi(1, i) = dxi(1, i) + acceleration_input(1, i)/m * ts;
            dxi(2, i) = dxi(2, i) + acceleration_input(2, i)/m * ts;
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
save("supremerrorsilvabrakes","supremum_reference_error");
title('Supremum Reference Error Over Time');
xlabel('Iterations');
ylabel('Supremum Error');
xlim([start_time+1,iterations])

subplot(2, 1, 2);
max_spacing_error = max(spacing_error(start_time:end,:), [], 2); % Find max error at each time step
save("silvabrakesspacing","max_spacing_error")

plot(start_time:iterations, max_spacing_error);
title('Maximum Spacing Error Over Time');
xlabel('Iterations');
ylabel('Maximum Spacing Error');
xlim([start_time+1, iterations]);
legend('Maximum Spacing Error');
    maxspacingerrortotalsilvabrakes = max(max_spacing_error);
    maxsupremreferrortotalsilvabrakes = max(supremum_reference_error);

    save(sprintf("supremum_error_platoon_size_%d_silva", N), 'maxsupremreferrortotalsilvabrakes');
    save(sprintf("spacing_error_platoon_size_%d_silva", N), 'maxspacingerrortotalsilvabrakes');
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
    plot(N, maxsupremreferrortotalsilvabrakes, 'ko', 'MarkerFaceColor', 'b');  % Black filled dot
    
    % Plot the vertical dotted line
    line([N N], [0 maxsupremreferrortotalsilvabrakes], 'LineStyle', '--', 'Color', 'b');  % Vertical line
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
    plot(N, maxspacingerrortotalsilvabrakes, 'ko', 'MarkerFaceColor', 'k');  % Black filled dot
    
    % Plot the vertical dotted line
    line([N N], [0 maxspacingerrortotalsilvabrakes], 'LineStyle', '--', 'Color', 'k');  % Vertical line
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