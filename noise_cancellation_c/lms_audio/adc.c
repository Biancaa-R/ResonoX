#include <stdio.h>
#include <stdlib.h>
#include <sndfile.h>

#define BUFFER_SIZE 1024  // Number of samples processed per iteration

int main(int argc, char *argv[]) {
    if (argc < 2) {
        printf("Usage: %s <input_audio.wav>\n", argv[0]);
        return 1;
    }

    SNDFILE *infile;
    SF_INFO sfinfo;
    
    // Open the audio file
    infile = sf_open(argv[1], SFM_READ, &sfinfo);
    if (!infile) {
        printf("Error: Unable to open file %s\n", argv[1]);
        return 1;
    }

    printf("Audio file details:\n");
    printf("Sample Rate: %d Hz\n", sfinfo.samplerate);
    printf("Channels: %d\n", sfinfo.channels);
    printf("Frames: %lld\n", sfinfo.frames);

    // Open the output file
    FILE *outfile = fopen("numaudio.txt", "w");
    if (!outfile) {
        perror("Error opening numaudio.txt");
        sf_close(infile);
        return 1;
    }

    // Write metadata to the file
    fprintf(outfile, "Sample Rate: %d Hz\n", sfinfo.samplerate);
    fprintf(outfile, "Channels: %d\n", sfinfo.channels);
    fprintf(outfile, "Frames: %lld\n\n", sfinfo.frames);

    // Buffer to store samples
    float buffer[BUFFER_SIZE];

    // Read samples and write to file
    sf_count_t readcount;
    while ((readcount = sf_readf_float(infile, buffer, BUFFER_SIZE)) > 0) {
        for (sf_count_t i = 0; i < readcount * sfinfo.channels; i++) {
            fprintf(outfile, "%f\n", buffer[i]);  // Write each sample
        }
    }

    printf("Conversion complete. Data saved to numaudio.txt\n");

    // Cleanup
    fclose(outfile);
    sf_close(infile);
    return 0;
}
