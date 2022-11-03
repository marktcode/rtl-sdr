# rtl-cw

Added to the existing repository a new code file rtl_cw.c. This is intended to perform the first steps of
signal conditioning: filtering, downsampling and digitization of CW morse. It has been tested on a pi 4 
operating with the RTL-SDR USB stick at default sampling rates of 2400000S/s and a default frequency in the ISM band.
The I/Q streams are initially filtered with a first order Butterworth LP with 3dB cuttoff at 600Hz.
This then down sampled to 48kHz where additional LP and HP filtering narrows the band further. The LP and HP
filters are 7th order and 2nd order Butterworth filters respectively also with 3dB cuttoff at 600Hz.
Thus the CW tone is presumed to be around ~550Hz. A optional experimental resonant-filter stage  with adjustable Q 
is also included in the filter chain with automatic centre-tuning at 550Hz. The output is then downsampled to 4800Hz
at which point the morse CW envelope is computed as the log(1+ sum of the squared filtered I & Q channels). 
The enevelope is further smoothed using a 200 tap gaussion FIR filter and the output is then resampled to a binary output
using a clock rate determined by the expected morse sending rate WPM. at 25WPM the  resampling is ~4.2Hz. This corresponds
to oversampling the 'dits' and 'dahs' by roughly a factor of 3. No attempt is made to synchronize with the send rate. 
This is done in the following stage.  rtl_cw  outputs both the binary stream and the signal envelope as UDP telemetry 
channels on ports 9222 and 9224 respectively. The pi is generally assumed to run headless and the user interface then
on a secodn host machine which recieves the UDP streams for output processing and display. 

The signal envelope may be displayed using a pertecs scope (setting this up will to be described later)

The final processing of the binary-to-message text is performed by another utility tc_mrsdcdr.c which is to be found in the 
tcode-tools repo. In effect the oversampled binary is treated as a stream to be decoded by a t-code decoder that uses a
variable-length self-synchronizing code set that automatically establishes synchronism with the 'dits' and 'dahs'. Because 
the send and receive timeing is not synchronized one observes a moire effect where the actual number of binary bits
corresponding to a 'dit' for example may not always be 3 but can vary... and together with timing jitter (such as 
is evident in the PiCW machine generated morse), channel effects, noise and bandlimited filtering effects their is 
a good deal of variation and uncertainty around the transition timing in the received morse envelope. Oversampling 
of the envelope ensures that the 'dits' and 'dahs' are captured but a dit is then presumed to comprise 2-5 x '1' bits 
and a dah '6-15' x '1's with the spacing similarly captured by runs of '0's. For practical reasons we extend the 'dit' 
sample range to anything between  1-5 x '1's.

The T-code decoder results in a stream of depletion codes that are then mapped in a 'many-to-one' relationship to 
the corresponding 'dits' and 'dahs'. And finally translated to morse letters for output as stdout. 

The result is a deterministic morse decoder that abstracts away the timing and synchronization challenges of interpretting 
the incoming morse CW.  The pair of utilities have under various conditions been tested on the pi over a range of word 
(10-200WPM) rates and noise and channel conditions with stable performance. However ultimate inclusion of the 200 tap 
gaussian filter now limits the WPM rate to around 50WPM.

