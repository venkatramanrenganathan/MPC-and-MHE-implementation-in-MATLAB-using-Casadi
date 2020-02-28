function [t0, x0, u0] = shift_with_noise(T, t0, x0, u,f)

% Create Noise vectors
a = 0;
b = 1;
thetaNoise = a + (b-a).*rand(1);
a = 0;
b = 0.1;
velocityNoise = a + (b-a).*rand(1);

controlNoise = [velocityNoise; thetaNoise];
controlNoise = [0;0];
a = 0;
b = 0.05;
systemNoise = a + (b-a).*rand(1);
systemNoise = [systemNoise; systemNoise; 0];


st = x0;
con = u(1,:)';
con = con + controlNoise;
f_value = f(st,con);
st = st+ (T*f_value) + systemNoise;
x0 = full(st);

t0 = t0 + T;
u0 = [u(2:size(u,1),:);u(size(u,1),:)];
end