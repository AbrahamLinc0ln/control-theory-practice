%% 1D Kalman Filter
% Uses constant velocity model to track position
% Marshall Trout
% Aug. 2017

% load data
data = %data file name goes here

dataLength = length(data);

% Setup matrices for update
T = 10;

SI = [1 T; 0 1];

St = eye(2); % State Covariance Matrix

M = [1 0]; % The observation matrix

Q = [0 0; 0 0.001]; % Q[2,2] correspons to dependence on model. Vary this relative to R
R = 1;

Xtt(:,1) = [0;0]; % Initial state estimate
Xt_1t_1 = Xtt(:,1);
St_1t_1 = St;


for i = 1:dataLength % loop through data
    Xtt_1 = SI*Xt_1t_1;
    Stt_1 = SI*St_1t_1*SI'+Q;
    Y = data(i);
    Kt = Stt_1*M'*inv(M*Stt_1*M'+R);
    Xtt(:,i+1) = Xtt_1 + Kt*(Y-M*Xtt_1); % Prediction
    Stt = (I - Kt*M)*Stt_1;
    Xt_1t_1 = Xtt(:,i+1);
    St_1t_1 = Stt;
    
end