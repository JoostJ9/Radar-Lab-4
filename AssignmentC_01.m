%% Angular resolution as funtion of angle
figure;
fc = 79e9
lambda = (3e8/fc)
theta = -90:0.1:90
ang_res = 0.886.*lambda./(4.*2.*lambda./2.*cos(theta))
plot(theta, ang_res)
xlabel("Angle (\circ)")
ylabel("Angular resolution (\circ)")
