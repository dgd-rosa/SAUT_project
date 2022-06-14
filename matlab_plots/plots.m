% Message structure
% pose.state.x\tpose.state.y\tpose.yaw\tnav_pos[0]\tnav_pos[1]\tnav_pos[2]\tpose.covariance\n

clc;
clear all;

file = fopen('medusa_stop.txt', 'r');
formatSpec = "%f\t%f\t%f\t%f\t%f\t%f\t[%f, %f, %f, %f, %f, %f, %f, %f, %f]\n";
data = fscanf(file, formatSpec, [15 Inf]); % Each column is a message

data(1,:) = data(1,:) - 4290794.43;
data(2,:) = data(2,:) - 491936.56;

data(4,:) = data(4,:) - 4290794.43;
data(5,:) = data(5,:) - 491936.56;

%t = length(data)
t = 1:length(data);


% figure(1);
% plot(t, data(1,:));
% xlabel('Iterations');
% ylabel('Position in X Axis [m]');
% % xlim([0 5/f]);
% % ylim([-2.5 2.5]);
% % yticks(linspace(-4, 4, 9))
% title("Tensão Triangular Adquirida");

figure(2);
plot(data(1,:), data(2,:),  'r*', data(4,:), data(5,:), 'g*');
%plot(data(4,:), data(5,:));
xlabel('Position in X Axis [m]');
ylabel('Position in Y Axis [m]');
% xlim([0 5/f]);
% ylim([-2.5 2.5]);
% yticks(linspace(-4, 4, 9))
title("Comparison between GroundTruth Position and EKF Position");
hold on;
pos = [data(1,1), data(2,1), 0]
cov_matrix = [data(7,1) data(8,1), data(9,1); data(10,1) data(11,1), data(12,1); data(13,1) data(14,1), data(15,1)];
% https://www.mathworks.com/matlabcentral/fileexchange/4705-error_ellipse
h = error_elipse( cov_matrix, 'conf', 0.99, 'mu', pos );
plot(h(1))



% 
% %transformada
% y = fft(data_cleaned);
% %Espetro bilateral
% P2 = abs(y/N);
% %Espetro Unilateral
% P1 = P2(1:N/2 + 1);
% P1(2:end-1) = 2*P1(2:end-1);
% F = fs*(0:(N/2))/N;
% 
% figure(2);
% plot(F,20*log10(P1));
% xlabel('Frequência [Hz]');
% ylabel('Amplitude do Espetro total[dBV]');
% xlim([0 fs/2])
% title("Espectro Unilateral");
% %ylim([-60 10]);
% yticks(linspace(-120, 20, 5))
% 

