%% Rigid Body Motion Tracker for the Public View
% Author: John Johnson
% Version: 6.2
% Last edited: August 29th 2026

% This is a MATLAB computer-vision system developed for undergrad research
% at Rutgers University under Professor Haim Baruh

% Tracks rigid body translation and rotation from video and calculates
% linear/angular kinematics, energy, and coefficient of restitution.

%% REQUIRED MATLAB ADD-ONS TO RUN: 
% - The Image Processing Toolbox by MathWorks
% - The Computer Vision Toolbox by MathWorks. 
% go to HOME -> Add-Ons -> Explore Add-Ons - Search
% It will break and give you an error if you dont have these.

%% Program Inputs:
% - To run this you need a Video of a moving object, object should be visible in the first frame.
% - The user then clicks 2 points in the video of known length, for example
% 1 meter, then enters that distance in the terminal below.
% - Afterwards you select the object & tracking begins. You have different
% options on the side such as to r e-select the object, stop the data
% collection early, or terminate the program. 

%% Program Outputs:
% - Position and orientation graphs & values over time.
% Takes the derivatives of those to find velocity, acceleration, omega, alpha, and
% there's also an option in settings to calculate energy and coefficient of
% restitution. 

% Cleans up the terminal and side windows, ignore this
clear; close all; clc;
disp("[SYSTEM] Starting " + mfilename + "..")

%% ENTER VIDEO NAME HERE
VideoFile = "BookVid.mp4";

% example:
%VideoFile = "MotionVid1.mp4";

% Then click run! 


%% Other Important Notes: 
% - You need to enter the file name of the video into the "VideoFile" string before starting so the
% program knows what video you'd like to run.
% - Videos need to be in your matlab files to be ran. 
% - If the video comes out rotated, tweak the "rotateAngle" variable
% below.
% - There's a lot of important variables below that affect the program. 

% --------------------------------------------------------------

%% Important Settings
save_data = false;
rotateAngle = -90;
SlowMotionFactor = 4;
SkipFrameFactor = 8;
useSGolay       = false;     % smooth non-collisio parts with SGolay
EnergyAnalysis = false; 

showVideo = true;

%% Plots
plotPosition     = true;
plotVelocity     = true;
plotAcceleration = true;
plotJerk         = true;     
plotTheta        = true;
plotOmegaAlpha   = true;
PlotObjectPoints = false;
PlotCollisions   = true; 

%% Other Boolean Settings
showTrajectoryOverlay  = true;
showOrientationOverlay = true;
showTextOverlay        = true;
showROIOverlay         = true;
checkVideoDimensions = false;
MeasureInDegrees = false;

%% Parameters
% Feature detection / tracking settings
% I honestly wouldn't touch this, these settings work well.
minQuality      = 0.0005;   % lower means more candidate feature points
maxPoints       = 10000;      % max number of feature points to keep
maxBidirErr     = inf;      % looser matching threshold
minPoints       = 50;       % below this, force ROI reselect

%% Smoothing settings for kinematics
smoothWin       = 9;       % used only if Savitzky-Golay is unavailable
sgolayOrder     = 3;
sgolayFrame     = 7;       % must be odd

%lower sgolayFrame -> more accurate graph but shakier
%higher sgolayFrame -> cleaner looking data but may smooth out spikes or
%important details

%% Collision Detection Constants
%CDT stands for collision detection threshold. Higher number means less collisions detected.
x_CDT = 5;
y_CDT = 5;
t_CDT = 5;
% "x", "y", "t" = x axis, y axis, theta


%% Motion filter (removes points that barely moved when object is clearly moving)
minMotionPx     = 1.5;
MaxMotionPx = 150;
useMotionFilter = true;
FilterPointsTooFar = true;

% If true, move the stored center using the rigid transform each frame
useTransformCenter = true;

%% Video metadata
vidMeta = VideoReader(VideoFile); % Gets video data
fps = vidMeta.FrameRate; %Gets framerate of video object

if fps <= 0 || ~isfinite(fps) %checks that fps is valid
    fps = 30;
    warning("[WARNING] Invalid FPS detected. Defaulting to 30 FPS")
end

if SlowMotionFactor <= 0 || ~isfinite(SlowMotionFactor) %checks our SloMo factor is valid
    error("[WARNING] SlowMotionFactor must be positive.");
end

if checkVideoDimensions %if this is true, displays video pixel dimensions
    disp("[INFO] Video dimensions (pixels): " + vidMeta.Width + ", " + vidMeta.Height);
end

% Effective timestep between analyzed frames
dt = 1 / (fps * SlowMotionFactor); %calculates dt based off video speed, very important


%% Main GUI window
mainFig = figure(100); % makes the main figure for our tracker's gui
clf(mainFig); %clears the figure
set(mainFig, "Name", mfilename, ... %Customizes the main background panel everything is on
    "NumberTitle", "off", ...
    "Color", [0.12 0.12 0.12]);

axTrack = axes("Parent", mainFig, ... %creates axTrack which is the gui obejct that will show out video
    "Units", "normalized", ...
    "Position", [0.05 0.10 0.72 0.84]);

statusBox = uicontrol(mainFig, "Style", "text", ... %Makes the status box opn the side (not rlly important tbh)
    "Units", "normalized", ...
    "Position", [0.80 0.89 0.17 0.06], ...
    "String", "Ready", ...
    "BackgroundColor", [0.15 0.15 0.15], ...
    "ForegroundColor", "w", ...
    "FontSize", 10, ...
    "FontWeight", "bold");

%% Buttons
uicontrol(mainFig, "Style", "pushbutton", "String", "Stop Early", ... %makes Stop early button
    "Units", "normalized", "Position", [0.80 0.79 0.17 0.07], ...
    "FontSize", 10, ...
    "Callback", @(~,~) setappdata(mainFig, "stopRequested", true));

uicontrol(mainFig, "Style", "pushbutton", "String", "Reselect ROI", ...%makes Reselect button
    "Units", "normalized", "Position", [0.80 0.69 0.17 0.07], ...
    "FontSize", 10, ...
    "Callback", @(~,~) setappdata(mainFig, "reselectRequested", true));

uicontrol(mainFig, "Style", "pushbutton", "String", "Terminate", ... %makes Terminate button
    "Units", "normalized", "Position", [0.80 0.59 0.17 0.07], ...
    "FontSize", 10, ...
    "Callback", @(~,~) terminateProgram(mainFig));

instructionsText = [ ... %instructions text 
    "Workflow" + newline + newline + ...
    "1. Click 2 calibration points" + newline + ...
    "2. Enter real length" + newline + ...
    "3. Draw freehand ROI around object" + newline + ...
    "4. Tracking starts" + newline + newline + ...
    "Buttons" + newline + ...
    "Stop Early = stop + save" + newline + ...
    "Reselect ROI = redraw object" + newline + ...
    "Terminate = exit no save" ...
];

uicontrol(mainFig, "Style", "text", ... %adds instructions text on the side
    "Units", "normalized", ...
    "Position", [0.80 0.10 0.17 0.32], ...
    "String", instructionsText, ...
    "BackgroundColor", [0.15 0.15 0.15], ...
    "ForegroundColor", "w", ...
    "FontSize", 9, ...
    "HorizontalAlignment", "left");

setappdata(mainFig, "stopRequested", false); %makes event if stop button is pressed
setappdata(mainFig, "reselectRequested", false); %event if reselect button is pressed
setappdata(mainFig, "terminateRequested", false); % event if terminate button is pressed

%% Open video
vid = VideoReader(VideoFile); %Open and save the video object

frame1 = readFrame(vid); %first frame of video
frame1 = imrotate(frame1, rotateAngle); %rotates the video if we want 

%% Calibration
cla(axTrack); %clears axTrack
imshow(frame1, "Parent", axTrack); %puts frame1 on axTrack
title(axTrack, "CALIBRATION: click TWO points on a known-length object"); % adds calibration instructions
set(statusBox, "String", "Calibration"); %changes status box to calibration

[xc, yc] = ginput(2); %2 inputs we click from calibration, xc and yc. this is a 2x2 matrix

% Distance between clicked calibration points in pixels
pxDist = hypot(xc(2)-xc(1), yc(2)-yc(1));

% Convert pixels to physical units
realLen = input("[INPUT] Enter the distance of the 2 points (meters): ");
mPerPx = realLen / pxDist; % Calculates Meters per Pixel for our video.

if EnergyAnalysis
    ObjectMass = input("[INFO] Enter object's mass (kg): ");
    ObjectMomentOfInertia = input("[INFO] Enter object's moment of inertia (kg/m^2): ");
end

if EnergyAnalysis
    pause(0.5);
    disp("---------------------------------------------------")
    disp("[INFO] VIDEO AND OBJECT DATA");
    disp("Meters per pixel: " + string(round(mPerPx,4)));
    disp("Mass: " + string(ObjectMass) + " kg");
    disp("Moment of Inertia: " + string(ObjectMomentOfInertia) + " kg/m^2");
else
    disp("[INFO]" + " Pixels per meter: " + string(round(mPerPx,4)));
end


%% Initial ROI
cla(axTrack); % clears axTrack again
imshow(frame1, "Parent", axTrack); % displays frame1 on axTrack again
title(axTrack, "TRACKING: draw around the object"); % titles to draw around the object
set(statusBox, "String", "Draw ROI"); % changes status box

% Freehand ROI is better than a box for pens, erasers, angled objects, etc.
roiHandle = drawfreehand(axTrack); % lets user draw a freehand ROI on the axes axTrack
roiMask = createMask(roiHandle); %makes a binary mask where pixels inside the region are marked as true, outside are marked as false
roiBoundary = roiHandle.Position; % returns boundary coordinates of the freehand ROI

roiW = max(roiBoundary(:,1)) - min(roiBoundary(:,1));
roiH = max(roiBoundary(:,2)) - min(roiBoundary(:,2));

% Initial stored center is the centroid of the user-drawn mask
stats = regionprops(roiMask, 'Centroid');

if ~isempty(stats)
    cx = stats(1).Centroid(1);
    cy = stats(1).Centroid(2);
else
    xMin = min(roiBoundary(:,1));
    xMax = max(roiBoundary(:,1));
    yMin = min(roiBoundary(:,2));
    yMax = max(roiBoundary(:,2));

    cx = (xMin + xMax) / 2;
    cy = (yMin + yMax) / 2;
end

%% Initial feature points
% Detect strong feature points only inside the freehand mask
objectPoints = detectPointsInMask(frame1, roiMask, minQuality, maxPoints);

if size(objectPoints,1) < 3
    error("[WARNING] Too few points detected. Tighten ROI or lower minQuality.");
end

%% Initialize point tracker
pointTracker = vision.PointTracker("MaxBidirectionalError", maxBidirErr);
initialize(pointTracker, objectPoints, frame1);

% prevPoints stores the point set from the previous processed frame
prevPoints = objectPoints;

%% Storage arrays
t = 0;
thetaCum = 0;

timeList     = [];
cx_px        = [];
cy_px        = [];
dThetaList   = [];
thetaCumList = [];
pointCount   = [];

set(statusBox, "String", "Tracking");

%% Main loop
while hasFrame(vid)

    frame = readFrame(vid);
    frame = imrotate(frame, rotateAngle);

    if getappdata(mainFig, "terminateRequested")
        disp("[SYSTEM] Run terminated. No files saved.");
        return
    end

    if getappdata(mainFig, "stopRequested")
        disp("[SYSTEM] Early stop requested.");
        break
    end

    if getappdata(mainFig, "reselectRequested")
        disp("[SYSTEM] Manual re-selection requested.");

        cla(axTrack);
        imshow(frame, "Parent", axTrack);
        title(axTrack, "RESELECT ROI: draw around the object");
        set(statusBox, "String", "Reselect ROI");

        roiHandle = drawfreehand(axTrack);
        roiMask = createMask(roiHandle);
        roiBoundary = roiHandle.Position;

        roiW = max(roiBoundary(:,1)) - min(roiBoundary(:,1));
        roiH = max(roiBoundary(:,2)) - min(roiBoundary(:,2));

        objectPoints = detectPointsInMask(frame, roiMask, minQuality, maxPoints);

        if size(objectPoints,1) < 3
            disp("[WARNING] Re-detection failed. Stopping.");
            break
        end

        release(pointTracker);
        pointTracker = vision.PointTracker("MaxBidirectionalError", maxBidirErr);
        initialize(pointTracker, objectPoints, frame);

        prevPoints = objectPoints;
        setappdata(mainFig, "reselectRequested", false);
        set(statusBox, "String", "Tracking");
        continue
    end

    %% Track points
    % newPoints are the predicted tracked point locations in the current frame
    % validity tells us which points were successfully tracked
    [newPoints, validity] = step(pointTracker, frame);

    % Only keep the matched points that survived tracking
    visiblePoints = newPoints(validity,:);
    prevVisible   = prevPoints(validity,:);

    %% Remove nearly stationary junk points
    % If the object is clearly moving, points that barely moved are often
    % background junk or weak mismatches.
    if useMotionFilter && ~isempty(visiblePoints)
        pointMotion = hypot(visiblePoints(:,1) - prevVisible(:,1), ...
                            visiblePoints(:,2) - prevVisible(:,2));

        medMotion = median(pointMotion);
        
        if medMotion > 2
            keep = pointMotion > minMotionPx & pointMotion < MaxMotionPx;
            visiblePoints = visiblePoints(keep,:);
            prevVisible   = prevVisible(keep,:);
        end

        if FilterPointsTooFar
            
            MaxDistFromCenterX = vidMeta.Width/5;
            MaxDistFromCenterY = vidMeta.Height/2;

            pointDistance = abs(visiblePoints(:,1) - abs(median(visiblePoints(:,1)))) ;
            keepGoodX = (pointDistance < MaxDistFromCenterX);
            visiblePoints = visiblePoints(keepGoodX, :);
            prevVisible = prevVisible(keepGoodX, :);

            pointYDistance = abs(visiblePoints(:,2) - abs (median(visiblePoints(:,2)))) ;
            keepGoodY = (pointYDistance < MaxDistFromCenterY);
            visiblePoints = visiblePoints(keepGoodY,: );
            prevVisible = prevVisible(keepGoodY, :);  
            
        end

    end

    n = size(visiblePoints,1);

    %% Auto reselect if too few points
    if n < minPoints
        disp("Too few good points left. Please redraw ROI.");

        cla(axTrack);
        imshow(frame, "Parent", axTrack);
        title(axTrack, "RESELECT ROI: draw around the object");
        set(statusBox, "String", "Reselect ROI");

        roiHandle = drawfreehand(axTrack);
        roiMask = createMask(roiHandle);
        roiBoundary = roiHandle.Position;

        roiW = max(roiBoundary(:,1)) - min(roiBoundary(:,1));
        roiH = max(roiBoundary(:,2)) - min(roiBoundary(:,2));

        objectPoints = detectPointsInMask(frame, roiMask, minQuality, maxPoints);

        if size(objectPoints,1) < 3
            disp("[WARNING] Re-detection failed. Stopping.");
            break
        end

        release(pointTracker);
        pointTracker = vision.PointTracker("MaxBidirectionalError", maxBidirErr);
        initialize(pointTracker, objectPoints, frame);

        prevPoints = objectPoints;
        set(statusBox, "String", "Tracking");
        continue
    end

    %% Rigid transform and center update
    dTheta = 0;

    % Need at least 3 corresponding points to estimate a 2D rigid transform
    if size(prevVisible,1) >= 3
        try
            tform = estimateGeometricTransform2D(prevVisible, visiblePoints, "rigid");

            % Extract incremental rotation from the transform matrix
            % Keep this as-is for now since this is your working version
            R = tform.T(1:2,1:2);
            dTheta = atan2(R(2,1), R(1,1));

            % Move the stored center using the estimated rigid transform
            if useTransformCenter && isfinite(cx) && isfinite(cy)
                [cx, cy] = transformPointsForward(tform, cx, cy);
            end

        catch
            dTheta = 0;
        end
    end

    % Emergency fallback only if the propagated center becomes invalid
    if ~isfinite(cx) || ~isfinite(cy)
        if ~isempty(visiblePoints)
            cx = median(visiblePoints(:,1));
            cy = median(visiblePoints(:,2));
        end
    end

    % Accumulate object angle over time
    thetaCum = thetaCum + dTheta;

    %% Store data
    timeList(end+1,1)     = t;
    cx_px(end+1,1)        = cx;
    cy_px(end+1,1)        = cy;
    dThetaList(end+1,1)   = dTheta;
    thetaCumList(end+1,1) = thetaCum;
    pointCount(end+1,1)   = n;

    %% Display
    if showVideo
        cla(axTrack);
        imshow(frame, "Parent", axTrack);
        hold(axTrack, "on");

        % Tracked feature points
        plot(axTrack, visiblePoints(:,1), visiblePoints(:,2), "g+");

        % Stored center point
        plot(axTrack, cx, cy, "ro", "MarkerSize", 8, "LineWidth", 2);

        % ROI outline the user drew
        if showROIOverlay && ~isempty(roiBoundary)
            boundaryClosed = [roiBoundary; roiBoundary(1,:)];
            plot(axTrack, boundaryClosed(:,1), boundaryClosed(:,2), "y-", "LineWidth", 1.2);
        end

        % Trajectory of tracked center
        if showTrajectoryOverlay && numel(cx_px) >= 2
            plot(axTrack, cx_px, cy_px, "c-", "LineWidth", 1.2);
        end

        % Orientation line based on accumulated rotation
        if showOrientationOverlay && isfinite(cx) && isfinite(cy)
            orientLen = 0.35 * max(roiW, roiH);
            x2 = cx + orientLen * cos(thetaCum);
            y2 = cy + orientLen * sin(thetaCum);
            plot(axTrack, [cx x2], [cy y2], "b-", "LineWidth", 2);
        end

        if showTextOverlay
            txt = sprintf("t = %.3f s\nPoints = %d", t, n);
            text(axTrack, 10, 20, txt, ...
                "Color", "w", ...
                "FontSize", 8, ...
                "FontWeight", "bold", ...
                "VerticalAlignment", "top", ...
                "BackgroundColor", "k", ...
                "Margin", 4);
        end

        hold(axTrack, "off");
        title(axTrack, sprintf("%s", VideoFile));
        drawnow;
    end

    %% Update tracker state
    setPoints(pointTracker, visiblePoints);
    prevPoints = visiblePoints;

    t = t + dt;
end

set(statusBox, "String", "Done");
disp("[SYSTEM] Tracking finished.");

%% Force consistent lengths
timeList     = timeList(:);
cx_px        = cx_px(:);
cy_px        = cy_px(:);
dThetaList   = dThetaList(:);
pointCount   = pointCount(:);
thetaCumList = thetaCumList(:);

N = min([numel(timeList), numel(cx_px), numel(cy_px), numel(dThetaList), numel(pointCount), numel(thetaCumList)]);
timeList     = timeList(1:N);
cx_px        = cx_px(1:N);
cy_px        = cy_px(1:N);
dThetaList   = dThetaList(1:N);
pointCount   = pointCount(1:N);
thetaCumList = thetaCumList(1:N);

%% Convert from pixels to meters
x_m = cx_px * mPerPx;
y_m = cy_px * mPerPx;

% Shift trajectory so the first point is the origin
x_m = x_m - x_m(1);
y_m = y_m - y_m(1);

%% Rotation signal
theta = unwrap(cumsum(dThetaList));

if MeasureInDegrees
    theta = rad2deg(theta);
end

%% Find Collision Info
[x_TimeOfCollisions, y_TimeOfCollisions, t_TimeOfCollisions, CollisionIndex_x, CollisionIndex_y, ...
    CollisionIndex_t] = findCollisionInfo(x_m,y_m,theta, dt, timeList, x_CDT, y_CDT, t_CDT);


%% Smooth signals before differentiating
% Savitzky-Golay usually gives cleaner derivatives than a plain moving average.

if useSGolay && (exist('sgolayfilt', 'file') == 2) && (sgolayFrame > sgolayOrder + 2)
    
    if mod(sgolayFrame,2) == 0
        sgolayFrame = sgolayFrame + 1;
    end

    if sgolayFrame > numel(x_m)
        sgolayFrame = max(3, 2 * floor((numel(x)-1)/2) + 1);
    end

    % Now here's the actual function
    x_s = ProcessData(x_m, CollisionIndex_x, sgolayOrder, sgolayFrame, timeList);
    y_s = ProcessData(y_m, CollisionIndex_y, sgolayOrder, sgolayFrame, timeList);
    theta_s = ProcessData(theta, CollisionIndex_t, sgolayOrder, sgolayFrame, timeList);
    disp("[SYSTEM] Data was smoothed using SGolay...")
else
    x_s = x_m;
    y_s = y_m;
    theta_s = theta;
    disp("[SYSTEM] Data is used raw and unsmoothed...")
end


%% Derivatives
vx = gradient(x_s, dt);
vy = gradient(y_s, dt);

ax = gradient(vx, dt);
ay = gradient(vy, dt);

jx = gradient(ax,dt);
jy = gradient(ay,dt);

omega = gradient(theta_s, dt);
alpha = gradient(omega, dt);

%% Calculate e values automatically
try
[e_values_y] = calculate_e_values(vy, timeList, CollisionIndex_y);
catch
    % Try catch incase we have no e_values
end


%% Save results
data = table(timeList, x_m, y_m, vx, vy, ax, ay, jx, jy, ...
          theta, omega, alpha, x_TimeOfCollisions, ...
          y_TimeOfCollisions, t_TimeOfCollisions, CollisionIndex_x, ...
          CollisionIndex_y, CollisionIndex_t, ...
    'VariableNames', {'time (s)','x_m (m)','y_m (m)','vx (m/s)','vy (m/s)','ax (m/s^2)','ay (m/s^2)', ...
    'j_x (m/s^3)', 'j_y (m/s^3)', 'theta_rad','omega_rads','alpha_rads', 'x_TimeOfCollisions (s)', ...
    'y_TimeOfCollisions (s)', 't_TimeOfCollisions (s)', 'CollisionIndex_x', 'CollisionIndex_y', ...
    'CollisionIndex_t'...
    });

if save_data
    writetable(data, VideoFile + "_results.csv");
    disp("[SUCCESS] Saved " + VideoFile + "_results.csv");
else
    disp("[SUCCESS] Data has not been saved.");
end

close(mainFig);

%% Plots
if plotPosition
    figure; plot(timeList, x_m); grid on; 
    xlabel("t (s)"); ylabel("x (m)"); title("x(t)");

    if PlotCollisions && ~isempty(x_TimeOfCollisions)
        hold on; L = xline(x_TimeOfCollisions,'Color', 'g', 'LineStyle', ':', 'LineWidth', 1);
        set(L, 'PickableParts', 'none');
    end

    figure; plot(timeList, y_m); grid on; 
    xlabel("t (s)"); ylabel("y (m)"); title("y(t)");

    if PlotCollisions && ~isempty(y_TimeOfCollisions)
        hold on; L = xline(y_TimeOfCollisions,'Color', 'c', 'LineStyle', ':', 'LineWidth', 1);
        set(L, 'PickableParts', 'none');
    end
    
    legend("x", "y")
    legend('AutoUpdate', 'off');
end

if plotVelocity
    figure; plot(timeList, vx); hold on; plot(timeList, vy); 
    grid on; xlabel("t (s)"); ylabel("v (m/s)"); title("Velocity");
    
    legend("vx","vy");
    legend('AutoUpdate', 'off');

    if PlotCollisions && (~isempty(x_TimeOfCollisions) | ~isempty(y_TimeOfCollisions))
        hold on; L1 = xline(x_TimeOfCollisions,'Color', 'g', 'LineStyle', ':', 'LineWidth', 1);
        set(L1, 'PickableParts', 'none');
        hold on; L2 = xline(CollisionIndex_y .* timeList,'Color', 'c', 'LineStyle', ':', 'LineWidth', 1);
        set(L2, 'PickableParts', 'none');
    end
end

if plotAcceleration
    figure; plot(timeList, ax); hold on; plot(timeList, ay); grid on;
    xlabel("t (s)"); ylabel("a (m/s^2)"); title("Acceleration"); legend("ax","ay");
    
    legend("ax", "ay")
    legend('AutoUpdate', 'off');

    if PlotCollisions && (~isempty(x_TimeOfCollisions) | ~isempty(y_TimeOfCollisions))
        hold on; L1 = xline(x_TimeOfCollisions,'Color', 'g', 'LineStyle', ':', 'LineWidth', 1);
        set(L1, 'PickableParts', 'none');
        hold on; L2 = xline(y_TimeOfCollisions,'Color', 'c', 'LineStyle', ':', 'LineWidth', 1);
        set(L2, 'PickableParts', 'none');
    end
end

if plotJerk
    n=5;
    figure; plot(timeList, jx); hold on; plot(timeList, jy); grid on;
    xlabel("t (s)"); ylabel("Jerk (m/s^3)"); title("Jerk"); 

    legend("jx","jy");
    legend('AutoUpdate', 'off');

    if PlotCollisions && (~isempty(x_TimeOfCollisions) | ~isempty(y_TimeOfCollisions))
        hold on; L1 = xline(x_TimeOfCollisions,'Color', 'g', 'LineStyle', ':', 'LineWidth', 1);
        set(L1, 'PickableParts', 'none');
        hold on; L2 = xline(y_TimeOfCollisions,'Color', 'c', 'LineStyle', ':', 'LineWidth', 1);
        set(L2, 'PickableParts', 'none');
    end
end

if plotTheta
    if MeasureInDegrees
        figure; plot(timeList, theta); grid on; xlabel("t (s)"); ylabel("\theta (deg)"); title("theta(t)");
    else
        figure; plot(timeList, theta); grid on; xlabel("t (s)"); ylabel("\theta (rad)"); title("theta(t)");
    end

    
    if PlotCollisions && ~isempty(t_TimeOfCollisions)
        hold on; L = xline(t_TimeOfCollisions,'Color', 'm', 'LineStyle', ':', 'LineWidth', 1);
        set(L, 'PickableParts', 'none');
    end

end

if plotOmegaAlpha
    figure; plot(timeList, omega); hold on; plot(timeList, alpha); grid on;
    xlabel("t (s)"); ylabel("rad/s, rad/s^2"); title("Angular kinematics");

    legend("omega","alpha");
    legend('AutoUpdate', 'off');

    if PlotCollisions && ~isempty(t_TimeOfCollisions)
        hold on; L = xline(t_TimeOfCollisions,'Color', 'm', 'LineStyle', ':', 'LineWidth', 1);
        set(L, 'PickableParts', 'none');
    end
end

if PlotObjectPoints
    figure; plot(timeList, pointCount, 'Color', 'green'); grid on; xlabel("t (s)"); ylabel("Points Tracked"); title("Point Count"); 
end


%% Helper functions
function objectPoints = detectPointsInMask(frame, mask, minQuality, maxPoints)

    if size(frame,3) == 3
        grayFrame = rgb2gray(frame);
    else
        grayFrame = frame;
    end
    grayFrame = imadjust(grayFrame);

    [rows, cols] = find(mask);
    if isempty(rows) || isempty(cols)
        objectPoints = zeros(0,2);
        return
    end

    rmin = max(min(rows), 1);
    rmax = min(max(rows), size(mask,1));
    cmin = max(min(cols), 1);
    cmax = min(max(cols), size(mask,2));

    grayCrop = grayFrame(rmin:rmax, cmin:cmax);
    maskCrop = mask(rmin:rmax, cmin:cmax);

    pts = detectMinEigenFeatures(grayCrop, "MinQuality", minQuality);

    if pts.Count < 1
        objectPoints = zeros(0,2);
        return
    end

    points = pts.Location;
    metrics = pts.Metric;

    x = round(points(:,1));
    y = round(points(:,2));

    x = max(1, min(x, size(maskCrop,2)));
    y = max(1, min(y, size(maskCrop,1)));

    inside = maskCrop(sub2ind(size(maskCrop), y, x));

    pointsInMask  = points(inside,:);
    metricsInMask = metrics(inside,:);

    if isempty(pointsInMask)
        objectPoints = zeros(0,2);
        return
    end

    [~, order] = sort(metricsInMask, "descend");
    order = order(1:min(maxPoints, numel(order)));
    pointsInMask = pointsInMask(order,:);

    objectPoints = pointsInMask;
    objectPoints(:,1) = objectPoints(:,1) + cmin - 1;
    objectPoints(:,2) = objectPoints(:,2) + rmin - 1;
end

%%
function [x_TimeOfCollisions, y_TimeOfCollisions, t_TimeOfCollisions, CollisionIndex_x, ...
    CollisionIndex_y, CollisionIndex_t] = findCollisionInfo(x, y, theta, dt, timeList, x_CDT, y_CDT, t_CDT)

x_CollisionsCount = 0;
y_CollisionsCount = 0;
t_CollisionsCount = 0;

MovMeanSmoothingNumber = 7;

%Derivatives
vx = gradient(x,dt);
vy = gradient(y,dt);
vt = gradient(theta,dt);
ax = gradient(vx,dt);
ay = gradient(vy,dt);
at = gradient(vt,dt);
jx = gradient(ax,dt);
jy = gradient(ay,dt);
jt = gradient(at,dt);

%Filtering Variables
testing_jx = movmean(abs(jx),MovMeanSmoothingNumber);
testing_jy = movmean(abs(jy),MovMeanSmoothingNumber);
testing_jt = movmean(abs(jt),MovMeanSmoothingNumber);
median_jx = median(testing_jx);
median_jy = median(testing_jy);
median_jt = median(testing_jt);

%Logical arrays for large jerk values for that variable
CollisionIndex_x = logical((testing_jx > median_jx*x_CDT));
CollisionIndex_y = logical((testing_jy > median_jy*y_CDT));
CollisionIndex_t = logical((testing_jt > median_jt*t_CDT));

%Checks how many collisions there are & at what times
x_TimeOfCollisions = zeros(length(timeList),1);
y_TimeOfCollisions = zeros(length(timeList),1);
t_TimeOfCollisions = zeros(length(timeList),1);

for n = 1:length(timeList)
    if (n + 1 <= length(timeList)) && (n-1 > 0)
        
        %check for x collisions
        if (CollisionIndex_x(n) == 1 && (vx(n)*vx(n+1)<0))
            x_CollisionsCount = x_CollisionsCount + 1;
            x_TimeOfCollisions(n) = timeList(n);
        end

        if (CollisionIndex_y(n) == 1 && (vy(n)*vy(n+1)<0))
            y_CollisionsCount = y_CollisionsCount + 1;
            y_TimeOfCollisions(n) = timeList(n);
        end
      
        if (CollisionIndex_t(n) == 1 && (vt(n)*vt(n+1)<0))
            t_CollisionsCount = t_CollisionsCount + 1;
            t_TimeOfCollisions(n) = timeList(n);
        end

    end
end

disp("[DATA] X collisions: " + x_CollisionsCount);
disp("[DATA] Y collisions: " + y_CollisionsCount);
disp("[DATA] Theta collisions: " + t_CollisionsCount);

end
%%

function [data_s] = ProcessData(data, CollisionIndex, sgolayOrder, sgolayFrame, timeList)
    
    data_s = data;

    for i = 1:length(CollisionIndex)-1

        if i == 1 && CollisionIndex(i) == 0
            startIdx = i;
        end

        if (CollisionIndex(i) == 1) && (CollisionIndex(i+1) == 0)
            startIdx = i+1;
            disp("startIdx = " + timeList(startIdx));
        end

        if (CollisionIndex(i) == 0) && (CollisionIndex(i+1) == 1) % Checks for start of collision
            endIdx = i;
            
            chunkSize = endIdx-startIdx + 1;

            if chunkSize > sgolayFrame
                data_s(startIdx:endIdx) = sgolayfilt(data(startIdx:endIdx), sgolayOrder, sgolayFrame);
            else
                tempsgolayFrame = floor(chunkSize/2)*2 +1;
                data_s(startIdx:endIdx) = sgolayfilt(data(startIdx:endIdx), sgolayOrder, tempsgolayFrame);
            end

        elseif (i == length(CollisionIndex)-1) && CollisionIndex(i+1) == 0 % Checks the end
            endIdx = i + 1;
            
            chunkSize = endIdx-startIdx +1;

            if chunkSize > sgolayFrame
                data_s(startIdx:endIdx) = sgolayfilt(data(startIdx:endIdx), sgolayOrder, sgolayFrame);
            else
                tempsgolayFrame = floor(chunkSize/2)*2 +1;
                data_s(startIdx:endIdx) = sgolayfilt(data(startIdx:endIdx), sgolayOrder, tempsgolayFrame);
            end
        end

    end
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
    disp("[SYSTEM] Detected # of e values: " + e_detected_count);
    if e_detected_count > 0
        disp(e_values);
    end
end

function terminateProgram(fig)
    setappdata(fig, "terminateRequested", true);
    close all;
end
