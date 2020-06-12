% Adaptive KF
% Marshall Trout
% Following the algorithm outlined in Akhlaghi 2017

clc;clear;close all;
%% Generate Data
t = (0.1:0.1:50);
trueData = [sawtooth(t) + t/10; zeros(1, length(t))];
corruptData = trueData + [0.75*randn(1, length(t)); zeros(1, length(t))];
figure(1)
hold on;
plot(t,trueData(1,:));
plot(t, corruptData(1,:));
legend('Original Signal','Signal with Noise')
hold off;


%% Common constant velocity KF (CKF)
% Initialize CKF

dt = 0.1;
F = [1 dt; 0 1]; % State transition matrix
H = [1 0]; % Observation matrix
Q = [0 0; 0 1]; % Process Noise Covariance
R = 1; % Measurement Noise Covariance
prioriX = [0;0]; % Initial State
posteriX = [0;0];
prioriP = eye(2); % State covariance matrix
posteriP = eye(2);

% Run CKF

for i=1:length(t)
    % Prediction
    prioriX = F*posteriX; % Predicted State Estimate
    prioriP = F*posteriP*F'+Q; % Predicted Error Covariance
    
    % Update
    err = corruptData(1,i) - H*prioriX; % Measurement Residual
    K = prioriP*H'*inv(R+H*prioriP*H'); % Kalman Gain
    posteriX = prioriX + K*err; % Update xhat
    posteriP = (eye(2) - K*H)*prioriP; % update P
    
    % Save xHat
    xHat(:,i) = posteriX;
end

% Plot result
figure(2);
hold on;
a = plot(t, xHat(1,:),'k');
b = plot(t, trueData(1,:),'r');
%c = plot(t, corruptData(1,:),'b');
hold off;

figure(3)
hold on;
a = plot(t, xHat(2,:),'k');
hold off;

%% Adaptive constant velocity KF (AKF)
% Initialize AKF

dt = 0.1;
alpha = 0.35; % Forgetting Factor
F = [1 dt; 0 1]; % State transition matrix
H = [1 0]; % Observation matrix
Q = [0 0; 0 1]; % Process Noise Covariance
R = 1; % Measurement Noise Covariance
prioriX = [0;0]; % Initial State
posteriX = [0;0];
prioriP = eye(2); % State covariance matrix
posteriP = eye(2);

% Run CKF

for i=1:length(t)
    % Prediction
    prioriX = F*posteriX; % Predicted State Estimate
    prioriP = F*posteriP*F'+Q; % Predicted Error Covariance
    
    % Update
    err = corruptData(1,i) - H*prioriX; % Residual between measurement and priori estimate
    K = prioriP*H'*inv(R+H*prioriP*H'); % Kalman Gain
    posteriX = prioriX + K*err; % Update xHat
    res = corruptData(1,i) - H*posteriX; % residual between measurement and posteri estimate
    R = alpha*R + (1-alpha)*(res*res'+H*prioriP*H'); % Update measurement noise covariance
    posteriP = (eye(2) - K*H)*prioriP; % Update P
    Q = alpha*Q+(1-alpha)*(K*err*err'*K'); % Update process noise covariance
    
    % Save xHat
    xHatAdaptive(:,i) = posteriX;
end

% Plot result
figure(4);
hold on;
a = plot(t, xHatAdaptive(1,:),'k');
b = plot(t, trueData(1,:),'r');
%c = plot(t, corruptData(1,:),'b');
hold off;

figure(5)
hold on;
a = plot(t, xHatAdaptive(2,:),'k');
hold off;

%% Compare
errorCommon = cumsum((trueData(1,:)-xHat(1,:)).^2);
errorAdaptive = cumsum((trueData(1,:)-xHatAdaptive(1,:)).^2);

figure(6)
hold on;
plot(t,errorCommon,'k');
plot(t,errorAdaptive,'r');
hold off;
