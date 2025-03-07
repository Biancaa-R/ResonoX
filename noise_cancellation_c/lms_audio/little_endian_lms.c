#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define N 128   // Number of filter coefficients
#define MU 0.01 // Step size

// WAV file header (44 bytes)
typedef struct {
    char chunkID[4];
    int chunkSize;
    char format[4];
    char subchunk1ID[4];
    int subchunk1Size;
    short audioFormat;
    short numChannels;
    int sampleRate;
    int byteRate;
    short blockAlign;
    short bitsPerSample;
    char subchunk2ID[4];
    int subchunk2Size;
} WAVHeader;

void adaptive_noise_cancellation(short *x, short *d, float *w, short *e, int length) {
    float y;

    for (int n = N - 1; n < length; n++) {
        y = 0.0;

        // Compute filter output (estimated noise)
        for (int i = 0; i < N; i++) {
            y += w[i] * x[n - i];
        }

        e[n] = d[n] - (short)y; // Error signal (clean audio)

        // Update filter weights
        for (int i = 0; i < N; i++) {
            w[i] += MU * e[n] * x[n - i];
        }
    }
}

int main() {
    FILE *noisyFile, *noiseFile, *outputFile;
    WAVHeader header;

    // Open noisy WAV file
    noisyFile = fopen("D:\\Downloads\\noise_cancellation_c\\lms_audio\\converted_audio.wav", "rb");
    noiseFile = fopen("noisy_audio.wav", "rb");

    if (!noisyFile || !noiseFile) {
        printf("Error opening input files!\n");
        return -1;
    }

    // Read and verify WAV header
    fread(&header, sizeof(WAVHeader), 1, noisyFile);
    fseek(noiseFile, 44, SEEK_SET); // Skip header for noise file

    // Ensure it's a valid WAV file
    if (strncmp(header.chunkID, "RIFF", 4) || strncmp(header.format, "WAVE", 4)) {
        printf("Invalid WAV file!\n");
        return -1;
    }

    // Calculate number of samples
    int length = header.subchunk2Size / (header.bitsPerSample / 8);

    // Allocate memory
    short *noisySignal = (short *)malloc(length * sizeof(short));
    short *noiseSignal = (short *)malloc(length * sizeof(short));
    short *filteredSignal = (short *)malloc(length * sizeof(short));
    float w[N] = {0}; // Adaptive filter weights

    // Read audio samples
    fread(noisySignal, sizeof(short), length, noisyFile);
    fread(noiseSignal, sizeof(short), length, noiseFile);

    fclose(noisyFile);
    fclose(noiseFile);

    // Apply LMS noise cancellation
    adaptive_noise_cancellation(noiseSignal, noisySignal, w, filteredSignal, length);

    // Update subchunk2Size for output file
    header.subchunk2Size = length * sizeof(short);
    header.chunkSize = 36 + header.subchunk2Size; // Total size

    // Write cleaned output as WAV
    outputFile = fopen("cleaned_audio.wav", "wb");
    fwrite(&header, sizeof(WAVHeader), 1, outputFile); // Write WAV header

    // Ensure little-endian order for samples
    for (int i = 0; i < length; i++) {
        short sample = filteredSignal[i];
        sample = (sample & 0xFF) << 8 | (sample >> 8); // Swap byte order
        fwrite(&sample, sizeof(short), 1, outputFile);
    }

    fclose(outputFile);

    // Cleanup
    free(noisySignal);
    free(noiseSignal);
    free(filteredSignal);

    printf("Noise removed! Output saved as 'cleaned_audio.wav'.\n");
    return 0;
}
