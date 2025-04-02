%% Angular resolution as funtion of angle
figure;
fc = 79e9
lambda = (3e8/fc)
theta = -89:0.1:89
ang_res = 0.886.*lambda./(4.*2.*lambda./2.*cos(theta*pi/180))
plot(theta, ang_res*180/pi)
xlabel("Angle (\circ)")
ylabel("Angular resolution (\circ)")
xlim([-80,80])