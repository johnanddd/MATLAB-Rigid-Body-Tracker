clc; close all; clear;
warning('off', 'all');

% Put your data here
data_name = "E3_3.5psi_1.0m_V6.mov" + "_results.csv";


disp("Attempting to graph " + data_name + "...");
data = readtable(data_name);

timeList = data{:,1};
x_m = data{:,2};
y_m = data{:,3};
vx = data{:,4};
vy = data{:,5};
ax = data{:,6};
ay = data{:,7};
jx = data{:,8};
jy = data{:,9};
theta = data{:,10};
omega = data{:,11};
alpha = data{:,12};
x_TimeOfCollisions = data{:,13};
y_TimeOfCollisions = data{:,14};
t_TimeOfCollisions = data{:,15};
CollisionIndex_x = data{:,16};
CollisionIndex_y = data{:,17};
CollisionIndex_t = data{:,18};

e_values_y = calculate_e_values(vy, timeList, CollisionIndex_y);

MeasureInDegrees = false;

%% Plots

    %% Position
    figure; plot(timeList, x_m); grid on; 
    xlabel("t (s)"); ylabel("x (m)"); title("x(t)");

    if true && ~isempty(x_TimeOfCollisions)
        hold on; L = xline(x_TimeOfCollisions,'Color', 'g', 'LineStyle', ':', 'LineWidth', 1);
        set(L, 'PickableParts', 'none');
    end

    figure; plot(timeList, y_m); grid on; 
    xlabel("t (s)"); ylabel("y (m)"); title("y(t)");

    if true && ~isempty(y_TimeOfCollisions)
        hold on; L = xline(y_TimeOfCollisions,'Color', 'c', 'LineStyle', ':', 'LineWidth', 1);
        set(L, 'PickableParts', 'none');
    end
    
    legend("x", "y")
    legend('AutoUpdate', 'off');


    %% Velocity
    figure; plot(timeList, vx); hold on; plot(timeList, vy); 
    grid on; xlabel("t (s)"); ylabel("v (m/s)"); title("Velocity");
    
    legend("vx","vy");
    legend('AutoUpdate', 'off');

    if true && (~isempty(x_TimeOfCollisions) | ~isempty(y_TimeOfCollisions))
        hold on; L1 = xline(x_TimeOfCollisions,'Color', 'g', 'LineStyle', ':', 'LineWidth', 1);
        set(L1, 'PickableParts', 'none');
        hold on; L2 = xline(CollisionIndex_y .* timeList,'Color', 'c', 'LineStyle', ':', 'LineWidth', 1);
        set(L2, 'PickableParts', 'none');
    end


    %% Acceleration
    figure; plot(timeList, ax); hold on; plot(timeList, ay); grid on;
    xlabel("t (s)"); ylabel("a (m/s^2)"); title("Acceleration"); legend("ax","ay");
    
    legend("ax", "ay")
    legend('AutoUpdate', 'off');

    if true && (~isempty(x_TimeOfCollisions) | ~isempty(y_TimeOfCollisions))
        hold on; L1 = xline(x_TimeOfCollisions,'Color', 'g', 'LineStyle', ':', 'LineWidth', 1);
        set(L1, 'PickableParts', 'none');
        hold on; L2 = xline(y_TimeOfCollisions,'Color', 'c', 'LineStyle', ':', 'LineWidth', 1);
        set(L2, 'PickableParts', 'none');
    end


    %% Jerk
    n=5;
    figure; plot(timeList, jx); hold on; plot(timeList, jy); grid on;
    xlabel("t (s)"); ylabel("Jerk (m/s^3)"); title("Jerk"); 

    legend("jx","jy");
    legend('AutoUpdate', 'off');

    if true && (~isempty(x_TimeOfCollisions) | ~isempty(y_TimeOfCollisions))
        hold on; L1 = xline(x_TimeOfCollisions,'Color', 'g', 'LineStyle', ':', 'LineWidth', 1);
        set(L1, 'PickableParts', 'none');
        hold on; L2 = xline(y_TimeOfCollisions,'Color', 'c', 'LineStyle', ':', 'LineWidth', 1);
        set(L2, 'PickableParts', 'none');
    end

    %% Theta
    if MeasureInDegrees
        figure; plot(timeList, theta); grid on; xlabel("t (s)"); ylabel("\theta (deg)"); title("theta(t)");
    else
        figure; plot(timeList, theta); grid on; xlabel("t (s)"); ylabel("\theta (rad)"); title("theta(t)");
    end

    
    if true && ~isempty(t_TimeOfCollisions)
        hold on; L = xline(t_TimeOfCollisions,'Color', 'm', 'LineStyle', ':', 'LineWidth', 1);
        set(L, 'PickableParts', 'none');
    end

    %% Omega & Alpha
    figure; plot(timeList, omega); hold on; plot(timeList, alpha); grid on;
    xlabel("t (s)"); ylabel("rad/s, rad/s^2"); title("Angular kinematics");

    legend("omega","alpha");
    legend('AutoUpdate', 'off');

    if true && ~isempty(t_TimeOfCollisions)
        hold on; L = xline(t_TimeOfCollisions,'Color', 'm', 'LineStyle', ':', 'LineWidth', 1);
        set(L, 'PickableParts', 'none');
    end

function [e_values] = calculate_e_values(v, timeList, collisionIndex)

    startIndex = 0;
    endIndex = 0;
    e_detected_count = 0;

    for i = 2:(length(collisionIndex)-1)
        if (collisionIndex(i) == 1) && (collisionIndex(i-1) == 0)
            startIndex = i;
        elseif (collisionIndex(i) == 0) && (collisionIndex(i-1) == 1)
            endIndex = i-1;
            v_1 = max(v(startIndex:endIndex));
            v_2 = min(v(startIndex:endIndex));
            e_value = abs(v_2/v_1);
            if e_value > 1
                e_value = 1/e_value;
            end
            e_detected_count = e_detected_count + 1;
            e_values(e_detected_count) = round(e_value,3);
        end
    end
    disp("Detected # of e values: " + e_detected_count);
    disp(e_values);
end


disp("Data successfuly loaded & graphed!")

