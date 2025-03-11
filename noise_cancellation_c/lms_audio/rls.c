#include <stdio.h>
#include <stdlib.h>
#include <math.h>
// WAV header structure
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
// Read WAV file
short *read_wav(const char *filename, WAVHeader *header, int *numSamples) {
    FILE *file = fopen(filename, "rb");
    if (!file) {
        printf("Error opening input file!\n");
        return NULL;
    }

    fread(header, sizeof(WAVHeader), 1, file);
    *numSamples = header->subchunk2Size / sizeof(short);
    
    short *data = (short *)malloc(*numSamples * sizeof(short));
    fread(data, sizeof(short), *numSamples, file);
    fclose(file);
    return data;
}

// Write WAV file
void write_wav(const char *filename, WAVHeader *header, short *data, int numSamples) {
    FILE *file = fopen(filename, "wb");
    if (!file) {
        printf("Error opening output file!\n");
        return;
    }

    fwrite(header, sizeof(WAVHeader), 1, file);
    fwrite(data, sizeof(short), numSamples, file);
    fclose(file);
}


int main(int argc, char *argv[]) {
    if (argc != 4) {
        printf("Usage: %s <desired_signal.wav> <reference_signal.wav> <output.wav>\n", argv[0]);
        return 1;
    }

    WAVHeader header;
    int numSamplesDesired, numSamplesNoise;

    // Read desired signal (clean speech)
    short *desired = read_wav(argv[1], &header, &numSamplesDesired);
    if (!desired) {
        fprintf(stderr, "Error: Failed to read desired signal from %s\n", argv[1]);
        return 1;
    }

    // Read reference noise signal
    short *reference = read_wav(argv[2], &header, &numSamplesNoise);
    if (!reference) {
        fprintf(stderr, "Error: Failed to read reference signal from %s\n", argv[2]);
        free(desired);  // Free previously allocated memory
        return 1;
    }

    // Ensure both signals have the same length
    if (numSamplesDesired != numSamplesNoise) {
        fprintf(stderr, "Error: Mismatched signal lengths!\n");
        free(desired);
        free(reference);
        return 1;
    }

    // Allocate memory for output signal
    short *output = (short *)malloc(numSamplesDesired * sizeof(short));
    if (!output) {
        fprintf(stderr, "Error: Memory allocation failed for output signal\n");
        free(desired);
        free(reference);
        return 1;
    }

    // Adaptive filtering using RLS algorithm
    double lambda = 0.99; // Forgetting factor
    double delta = 0.01;  // Initialization parameter

    int filterOrder = 32; // Order of the adaptive filter
    double *weights = (double *)calloc(filterOrder, sizeof(double));
    double *buffer = (double *)calloc(filterOrder, sizeof(double));
    double *P = (double *)malloc(filterOrder * filterOrder * sizeof(double));

    if (!weights || !buffer || !P) {
        fprintf(stderr, "Error: Memory allocation failed for RLS parameters\n");
        free(desired);
        free(reference);
        free(output);
        free(weights);
        free(buffer);
        free(P);
        return 1;
    }

    // Initialize P matrix as identity
    for (int i = 0; i < filterOrder; i++) {
        for (int j = 0; j < filterOrder; j++) {
            P[i * filterOrder + j] = (i == j) ? (1.0 / delta) : 0.0;
        }
    }

    // RLS Filtering Process
    for (int n = 0; n < numSamplesDesired; n++) {
        // Shift buffer
        for (int k = filterOrder - 1; k > 0; k--) {
            buffer[k] = buffer[k - 1];
        }
        buffer[0] = reference[n];

        // Compute output
        double y = 0.0;
        for (int k = 0; k < filterOrder; k++) {
            y += weights[k] * buffer[k];
        }

        // Compute error signal
        double error = desired[n] - y;
        output[n] = (short)round(error);

        // Compute gain vector K
        double *K = (double *)malloc(filterOrder * sizeof(double));
        double den = lambda;
        for (int k = 0; k < filterOrder; k++) {
            den += buffer[k] * P[k * filterOrder + k] * buffer[k];
        }
        for (int k = 0; k < filterOrder; k++) {
            K[k] = 0;
            for (int j = 0; j < filterOrder; j++) {
                K[k] += P[k * filterOrder + j] * buffer[j];
            }
            K[k] /= den;
        }

        // Update weight vector
        for (int k = 0; k < filterOrder; k++) {
            weights[k] += K[k] * error;
        }

        // Update inverse correlation matrix P
        double *P_temp = (double *)malloc(filterOrder * filterOrder * sizeof(double));
        for (int i = 0; i < filterOrder; i++) {
            for (int j = 0; j < filterOrder; j++) {
                P_temp[i * filterOrder + j] = P[i * filterOrder + j] - K[i] * buffer[j] * P[j * filterOrder + i];
            }
        }
        for (int i = 0; i < filterOrder * filterOrder; i++) {
            P[i] = (P_temp[i] + P_temp[i]) / (2.0 * lambda); // Stabilization
        }

        free(K);
        free(P_temp);
    }

    // Write output to a WAV file
    write_wav(argv[3], &header, output, numSamplesDesired);

    // Free allocated memory
    free(desired);
    free(reference);
    free(output);
    free(weights);
    free(buffer);
    free(P);

    printf("RLS Noise Cancellation completed. Output saved to %s\n", argv[3]);
    return 0;
}
