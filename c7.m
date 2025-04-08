clear; clc;

%% Radar Parameters
c = 3e8;
fc = 77e9;
lambda = c / fc;
B = 4e9;                   % 4 GHz bandwidth
T_chirp = 60e-6;           % Chirp duration
fs = 20e6;                 % Sampling rate
t = 0:1/fs:T_chirp-1/fs;   % Time vector

%% Array Configuration
d = lambda/2;
Tx = 2; Rx = 4;
N_virtual = Tx * Rx;
virtual_pos = d * (0:N_virtual-1);

%% Target Parameters
target_ranges = [2, 2];           % in meters
target_angles = [-10, 10];          % in degrees
target_rcs = [1, 1];              % relative amplitudes

%% Generate LFM Upchirp
f0 = fc;
k = B / T_chirp;
s_tx = exp(1j * 2 * pi * (f0 * t + 0.5 * k * t.^2));  % LFM upchirp

%% Simulate Received Signal per Virtual Channel
rx_signal = zeros(length(t), N_virtual);

for tgt = 1:length(target_ranges)
    R = target_ranges(tgt);
    theta = target_angles(tgt);
    amp = target_rcs(tgt);

    % Time delay and steering vector
    tau = 2 * R / c;
    delay_samples = round(tau * fs);
    a_theta = exp(1j * 2*pi * virtual_pos * sind(theta) / lambda).';  % (N_virtual x 1)

    % Create delayed signal
    delayed = zeros(size(t));
    if delay_samples < length(t)
        delayed(delay_samples+1:end) = s_tx(1:end - delay_samples);
    end

    % Add target contribution
    rx_signal = rx_signal + amp * delayed.' * a_theta.';
end

%% Mix with conjugate of transmitted chirp (beat signal)
beat_signal = rx_signal .* conj(repmat(s_tx.', 1, N_virtual));

%% FFT over fast-time (range FFT)
N_fft = 2048;
range_profile = fft(beat_signal, N_fft, 1);  % FFT over time
range_bins = (0:N_fft-1) * c / (2*B);

% Use strongest bin across channels
[~, bin_idx] = max(abs(sum(range_profile, 2)));
snapshot_full = range_profile(bin_idx, :).';  % N_virtual x 1

%% --- FULL ARRAY (8 virtual elements) ---
theta_scan = -90:0.1:90;
P_full = zeros(size(theta_scan));
steer_full = @(theta) exp(1j*2*pi*virtual_pos.'*sind(theta)/lambda);

for i = 1:length(theta_scan)
    a = steer_full(theta_scan(i));
    P_full(i) = abs(a' * snapshot_full)^2;
end

% Plot full array
figure;
plot(theta_scan, 10*log10(P_full / max(P_full)), 'LineWidth', 1.5); grid on;
xlabel('Azimuth angle (deg)'); ylabel('Normalized Power (dB)');
title('Azimuth Estimation (2 Tx × 4 Rx = 8 Virtual Channels)');

[~, locs_full] = findpeaks(P_full, 'SortStr', 'descend', 'NPeaks', 2);
disp('Estimated azimuths (full array):');
disp(theta_scan(locs_full));

%% --- REDUCED ARRAY (1 Tx × 4 Rx = 4 virtual elements) ---
% Select only Tx1 (virtual elements 1 to 4)
snapshot_tx1 = snapshot_full(1:4);
virtual_pos_tx1 = virtual_pos(1:4);

steer_tx1 = @(theta) exp(1j * 2*pi * virtual_pos_tx1.' * sind(theta)/lambda);
P_tx1 = zeros(size(theta_scan));

for i = 1:length(theta_scan)
    a = steer_tx1(theta_scan(i));
    P_tx1(i) = abs(a' * snapshot_tx1)^2;
end

% Plot reduced array
figure;
plot(theta_scan, 10*log10(P_tx1 / max(P_tx1)), 'LineWidth', 1.5); grid on;
xlabel('Azimuth angle (deg)'); ylabel('Normalized Power (dB)');
title('Azimuth Estimation (1 Tx × 4 Rx = 4 Virtual Channels)');

[~, locs_tx1] = findpeaks(P_tx1, 'SortStr', 'descend', 'NPeaks', 2);
disp('Estimated azimuths (Tx1 only):');
disp(theta_scan(locs_tx1));
