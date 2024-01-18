filename = "raw_values_rotating.csv";
% filename = "raw_values.csv"
M = csvread(filename, 1, 0);
x = M(:, 1);
y_values = M(:, 2);
z_values = M(:, 3);


Signal = x;

n = 200;                    % Filter order
f = [0 5 8 500] / 500;         % Frequency band edges
a = [1  1  0 0];           % Amplitudes
b = firpm(n,f,a);



Fs = 1000;
t = 0: 1/Fs:length(Signal)/Fs-(1/Fs);


Signal = filter(b, Signal);
Signal
N = length(Signal);
Signal = Signal - mean(Signal);                                            % to remove the frequency at 0 (or D-C offset)
sigFFT = (abs(fft(Signal)).^2);                  
bin_vals = 0 : N-1;
fax_Hz = bin_vals*Fs/N;
N_2 = ceil(N/2);
power = sigFFT(1:N_2);                                                     % get magnitude 
freq = fax_Hz(1:N_2);                                                      % get freq in Hz
subplot(3,1,2); plot(freq,power,'k')
xlabel('Frequency (Hz)','FontWeight','bold','FontSize',10)
ylabel('Power','FontWeight','bold','FontSize',10)
xlim([0 35]); title('Periodogram')

X = Signal - mean(Signal);
nfft = 512;                                                                % next larger power of 2
y = fft(X,nfft);                                                           % Fast Fourier Transform
y = abs(y.^2);                                                             % raw power spectrum density
y = y(1:1+nfft/2);                                                         % half-spectrum
[v,k] = max(y);                                                            % find maximum
f_scale = (0:nfft/2)* Fs/nfft;                                             % frequency scale
subplot(3,1,3);
plot(f_scale, y),axis('tight'),grid('on'),title('Dominant Frequency')
xlim([0 35]);




%plot(abs(real(fft(x))));

t = 0:0.001:1.035;
%subplot(2, 1, 1);
%plot(t, x, 'b-')
%title('x vs. t', 'FontSize', fontSize);
%xlabel('t', 'FontSize', fontSize);
%ylabel('x', 'FontSize', fontSize);
%grid on;
%y=fftshift(fft(x - mean(x)));
%f = (0:length(y)-1)*(10000)/length(y);
%subplot(2, 1, 2);
%plot(abs(y), 'b-', 'LineWidth', 2)
%grid on;
%title('Spectral Power', 'FontSize', fontSize);
%xlabel('frequency', 'FontSize', fontSize);
%ylabel('Power', 'FontSize', fontSize);
% Enlarge figure to full screen.
%set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0.04, 1, 0.96]);
