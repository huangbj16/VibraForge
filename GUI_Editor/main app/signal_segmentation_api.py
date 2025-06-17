import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft, fftfreq
from scipy.signal import stft
from scipy.signal import hilbert

DOMINANT_CUTOFF = 110

class signal_segmentation_api:
    def __init__(self):
        pass

    '''
    the function takes in a high sample rate produce signal, extract high frequency components using STFT, and extract low frequency components using Hilbert transform
    Input: 
        product_signal: the input signal with very high sample rate
        sample_rate: the sample rate of the input signal, e.g., 44100 Hz
        downsample_rate: the expected output sample rate for vibration commands,, e.g., 200 Hz
    Output: 
        high_freq_signal: a numpy array of the high frequency components (range [100, 400]), used for setting the frequency of vibration commands (find the nearest frequency out of the eight options)
        low_freq_signal: a numpy array of the low frequency components (range [0, 1]), used for setting the intensity of vibration commands (map to [0, 15])
    '''

    # def signal_segmentation(self, product_signal, sampling_rate, downsample_rate, threshold=110):
    #     """For test only, a constant of 0.5 for low and a decending from 500 to 0 for high"""
    #     # Generate a constant 0.5 for the low frequency signal
    #     low_freq_signal = np.full_like(product_signal, 0.5)
        
    #     # Generate a descending signal from 500 to 0 for the high frequency signal
    #     high_freq_signal = np.linspace(500, 0, len(product_signal))

    #     # Ensure the return types are NumPy arrays
    #     high_freq_signal = np.array(high_freq_signal)
    #     low_freq_signal = np.array(low_freq_signal)

    #     return high_freq_signal, low_freq_signal


    def signal_segmentation(self, product_signal, sampling_rate, downsample_rate, threshold=102):
        # Perform STFT on the signal to get the high-frequency components
        frequencies, times, Zxx = stft(product_signal, fs=sampling_rate, nperseg=int(20*sampling_rate/downsample_rate), noverlap=int(19*sampling_rate/downsample_rate))
        max_freq = np.argmax(np.abs(Zxx), axis=0)
        high_freq_signal = frequencies[max_freq][:-1]

        # Perform Hilbert transform on the signal to get the low-frequency components
        analytic_signal = hilbert(product_signal)
        amplitude_envelope = np.abs(analytic_signal)

        # Perform FFT on the envelope to get frequency components
        fft_envelope = fft(amplitude_envelope)
        fft_freq_env = fftfreq(len(fft_envelope), 1 / sampling_rate)

        # Use median frequency instead of max to compare with the threshold
        median_frequency = np.median(high_freq_signal)
        if median_frequency < threshold:
            # Set low frequency signal to the original and high frequency to zeros
            low_freq_signal = np.abs(product_signal)
            high_freq_signal = np.zeros_like(product_signal)
        else:
            # Filter out frequency components above downsample_rate/2 and IFFT back to time domain
            fft_envelope_filtered = fft_envelope.copy()
            fft_envelope_filtered[(fft_freq_env > downsample_rate//2)] = 0
            fft_envelope_filtered[(fft_freq_env < -downsample_rate//2)] = 0
            filtered_signal = np.real(np.fft.ifft(fft_envelope_filtered))

            # Clamp the low frequency signal between 0 and 1
            low_freq_signal = np.clip(filtered_signal, 0, 1)

            # Upsample both signals to match the original signal length using cubic interpolation
            high_freq_signal = np.interp(np.arange(len(product_signal)), np.linspace(0, len(product_signal)-1, len(high_freq_signal)), high_freq_signal)
            low_freq_signal = np.interp(np.arange(len(product_signal)), np.linspace(0, len(product_signal)-1, len(low_freq_signal)), low_freq_signal)

        # Ensure the return types are NumPy arrays
        high_freq_signal = np.array(high_freq_signal)
        low_freq_signal = np.array(low_freq_signal)

        return high_freq_signal, low_freq_signal



    # def signal_segmentation(self, product_signal, sampling_rate, downsample_rate):
    #     """Purely No Downsample Version"""
    #     # Perform STFT on the signal to get the high frequency components
    #     frequencies, times, Zxx = stft(product_signal, fs=sampling_rate, nperseg=int(sampling_rate))
        
    #     # Find the dominant frequency over all segments
    #     max_freq = np.argmax(np.abs(Zxx), axis=0)
    #     dominant_frequency = np.median(frequencies[max_freq])  # Use median for stability
        
    #     # Check if the dominant frequency is below the threshold (100 Hz)
    #     if dominant_frequency < 100:
    #         high_freq_signal = np.zeros(len(times))  # Set high frequency signal to zero
    #         low_freq_signal = np.abs(np.array(product_signal))  # Leave the low-frequency signal as the original
    #     else:
    #         # Perform Hilbert transform on the signal to get the low-frequency components
    #         high_freq_signal = np.full(len(times), dominant_frequency)  # Set constant frequency if above threshold
            
    #         analytic_signal = hilbert(product_signal)
    #         amplitude_envelope = np.abs(analytic_signal)
            
    #         # Perform FFT on the envelope to get frequency components
    #         fft_envelope = fft(amplitude_envelope)
    #         fft_freq_env = fftfreq(len(fft_envelope), 1 / sampling_rate)
            
    #         # Filter out frequency components above 100 Hz and ifft back to time domain
    #         fft_envelope_filtered = fft_envelope.copy()
    #         fft_envelope_filtered[(fft_freq_env > 100)] = 0
    #         fft_envelope_filtered[(fft_freq_env < -100)] = 0
    #         filtered_signal = np.real(np.fft.ifft(fft_envelope_filtered))
            
    #         # Processed low-frequency signal
    #         low_freq_signal = filtered_signal

    #     return high_freq_signal, low_freq_signal


    # def signal_segmentation(self, product_signal, sampling_rate, downsample_rate):
    #     # Set high frequency signal to 200 while keeping the same format and type
    #     high_freq_signal = np.full_like(product_signal, 145.0)

    #     # Keep the original product_signal as the low frequency signal, ensure it's a NumPy array
    #     low_freq_signal = np.array(product_signal)  # Convert to NumPy array if it isn't already

    #     return high_freq_signal, low_freq_signal



    # def signal_segmentation(self, product_signal, sampling_rate, downsample_rate):
    #     # Perform STFT on the signal to get the high frequency components
    #     frequencies, times, Zxx = stft(product_signal, fs=sampling_rate, nperseg=2*int(sampling_rate/downsample_rate))
    #     max_freq = np.argmax(np.abs(Zxx), axis=0)
    #     high_freq_signal = frequencies[max_freq][:-1]

    #     # Perform Hilbert transform on the signal to get the low frequency components
    #     analytic_signal = hilbert(product_signal)
    #     amplitude_envelope = np.abs(analytic_signal)
    #     # Perform FFT on the envelope to get frequency components
    #     fft_envelope = fft(amplitude_envelope)
    #     fft_freq_env = fftfreq(len(fft_envelope), 1 / sampling_rate)
    #     # filter out frequency components above 100 Hz, and ifft back to time domain
    #     fft_envelope_filtered = fft_envelope.copy()
    #     fft_envelope_filtered[(fft_freq_env > downsample_rate//2)] = 0
    #     fft_envelope_filtered[(fft_freq_env < -downsample_rate//2)] = 0
    #     filtered_signal = np.real(np.fft.ifft(fft_envelope_filtered))
    #     # Keep original signal, no downsampling
    #     low_freq_signal = filtered_signal

    #     return high_freq_signal, low_freq_signal


# Example usage
if __name__ == '__main__':
    wav_filename = 'square5_sine200.wav'
    # read the wav file, print the sampling rate and duration, extract the first second of the signal as product_signal
    import soundfile as sf
    signal, fs = sf.read(wav_filename)
    print(f'Sampling rate: {fs} Hz')
    print(f'Duration: {len(signal)/fs} seconds')
    product_signal = signal[:5*fs]

    # Parameters
    sampling_rate = fs  # in Hz
    duration = 5  # in seconds

    # Initialize the signal segmentation API
    signal_segmentation = signal_segmentation_api()

    # Perform signal segmentation
    downsample_rate = 200
    downsample_t = np.linspace(0, duration, len(product_signal), endpoint=False)
    
    high_freq_signal, low_freq_signal = signal_segmentation.signal_segmentation(product_signal, sampling_rate, downsample_rate)
    print(len(downsample_t), len(high_freq_signal), len(low_freq_signal))

    # Plot the high frequency signal
    plt.figure(figsize=(10, 4))
    plt.plot(downsample_t[:len(high_freq_signal)], high_freq_signal, '-o')
    plt.title('High Frequency Signal')
    plt.xlabel('Time [s]')
    plt.ylabel('Amplitude')
    plt.show()

    # Plot the low frequency signal
    plt.figure(figsize=(10, 4))
    plt.plot(downsample_t[:len(low_freq_signal)], low_freq_signal, '-o')
    plt.title('Low Frequency Signal')
    plt.xlabel('Time [s]')
    plt.ylabel('Amplitude')
    plt.show()
