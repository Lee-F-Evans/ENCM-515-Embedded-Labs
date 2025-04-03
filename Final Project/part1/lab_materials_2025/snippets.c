/* This is the start address of where we are going to put our input data */
#define DATA_START_ADDRESS	0x08020000

/* For this lab, we should know in advance how many samples we have to process... */
#define NUMBER_OF_SAMPLES	22050

/* This is a pointer to our input data */
float32_t* data = DATA_START_ADDRESS;

/* Here are our filter taps (as float32_t) */
static float32_t filter_taps[31] = {
  -0.004289844297235351,
  -0.00014156432081732112,
  -0.0036201686224482756,
  0.004827257985150652,
  0.0036155531164272516,
  0.020675517860072025,
  0.02402285521584296,
  0.051424412549522944,
  0.05939430591101754,
  0.09527637902148803,
  0.10443208231700706,
  0.14294541181873402,
  0.14707868991005837,
  0.18028398272481666,
  0.17316624396732433,
  0.19445415344945507,
  0.17316624396732433,
  0.18028398272481666,
  0.14707868991005837,
  0.14294541181873402,
  0.10443208231700706,
  0.09527637902148803,
  0.05939430591101754,
  0.051424412549522944,
  0.02402285521584296,
  0.020675517860072025,
  0.0036155531164272516,
  0.004827257985150652,
  -0.0036201686224482756,
  -0.00014156432081732112,
  -0.004289844297235351
};

/* Here are our filter taps (as int16_t or q15_t) */
static int16_t filter_taps[31] = {-141, -5, -119, 158, 118, 677, 787, 1685, 1946, 3122, 3422, 4684, 4819, 5908, 5674, 6372, 5674, 5908, 4819, 4684, 3422, 3122, 1946, 1685, 787, 677, 118, 158, -119, -5, -141};

/* let's have our history and output arrays defined somewhere */
float32_t history[31];
float32_t newdata[NUMBER_OF_SAMPLES];

/* this the function prototype for initializing the filter (specifically the history) */
void FloatFilterInit(void);

/* This is the function prototype that does the filtering given a new sample.
 * The filter should shuffle the history values (which it can because history is global 
 * This function should return a new output
 */
float32_t FloatFilterGet(float new_sample);


/* here is the function that does the filtering */
float32_t FloatFilterGet(float32_t newsample) {
	// set the new sample as the head
	history[0] = newsample;

	// set up and do our convolution

	// shuffle the history along for the next one?

	return accumulator;
}