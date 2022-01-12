%% Causal 2nd Order Savitzky Golay FIR Filter Coefficients Generator
% Marshall Trout
% 2019

function [A, filt] = causal_SGFilter(FrameLength, ts)
% FrameLength = length of filter
% ts = sample time
% filt(2,:) = FIR coeffs for derivative of filtered signal
% filt(3,:) = FIR coeffs for filtered signal
% The filter rows can be used directly in a Simulink FIR block

for i = (FrameLength-1):-1:0
    A(1,i+1) = (ts*i^2);
    A(2,i+1) = -1*i*ts;
end
A(3,:) = ones(FrameLength,1);
A = A';

filt = inv(A'*A)*A';
end