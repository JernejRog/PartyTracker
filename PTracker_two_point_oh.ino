#define EIDSP_QUANTIZE_FILTERBANK   0

#include <PDM.h>
#include <partytracker_16_inference.h>
#include <ArduinoBLE.h>

typedef struct {
  int16_t *buffer;
  uint8_t buf_ready;
  uint32_t buf_count;
  uint32_t n_samples;
} inference_t;

static inference_t inference;
static signed short sampleBuffer[2048];
static bool debug_nn = false;

void updateBLE();

BLEService micService("180F");
BLEUnsignedCharCharacteristic micServiceChar("2A19", BLERead | BLENotify);

int prevMicMode = 0;
int currentMicMode = 0;

const int numReadings = 10;
int voice[numReadings];
int music[numReadings];
int silent[numReadings];
int readIndex = 0;
int voiceTotal = 0;
int musicTotal = 0;
int silentTotal = 0;
int voiceAverage = 0;
int musicAverage = 0;
int silentAverage = 0;

uint64_t currMillis = 0;
uint64_t prevMillisRecord = 0;
uint64_t prevMillisBLEupdate=0;
int readingTime=1000;
uint32_t BLEupdatePeriod=10000;

void setup()
{
  Serial.begin(115200);

  ei_printf("Inferencing settings:\n");
  ei_printf("\tInterval: %.2f ms.\n", (float)EI_CLASSIFIER_INTERVAL_MS);
  ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
  ei_printf("\tSample length: %d ms.\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT / 16);
  ei_printf("\tNo. of classes: %d\n", sizeof(ei_classifier_inferencing_categories) / sizeof(ei_classifier_inferencing_categories[0]));

  if (microphone_inference_start(EI_CLASSIFIER_RAW_SAMPLE_COUNT) == false) {
    ei_printf("ERR: Failed to setup audio sampling\r\n");
    return;
  }

  while (!BLE.begin());
  BLE.setLocalName("MICMonitor");
  BLE.setAdvertisedService(micService);
  micService.addCharacteristic(micServiceChar);
  BLE.addService(micService);
  micServiceChar.writeValue(prevMicMode);
  BLE.advertise();

  for (int i = 0; i < numReadings; i++)
  {
    voice[i] = 0;
    music[i] = 0;
    silent[i] = 0;
  }
}

void loop()
{
  BLEDevice central = BLE.central();
  if (central)
  {
    while (central.connected())
    {
      updateBLE();

      currMillis = millis();

      if (currMillis - prevMillisRecord > readingTime)
      {
        bool m = microphone_inference_record();
        if (!m) {
          ei_printf("ERR: Failed to record audio...\n");
          return;
        }

        signal_t signal;
        signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
        signal.get_data = &microphone_audio_signal_get_data;
        ei_impulse_result_t result = { 0 };

        EI_IMPULSE_ERROR r = run_classifier(&signal, &result, debug_nn);
        if (r != EI_IMPULSE_OK) {
          ei_printf("ERR: Failed to run classifier (%d)\n", r);
          return;
        }
        voiceTotal = voiceTotal - voice[readIndex];
        musicTotal = musicTotal - music[readIndex];
        silentTotal = silentTotal - silent[readIndex];

        voice[readIndex] = result.classification[2].value * 1000;
        music[readIndex] = result.classification[0].value * 1000;
        silent[readIndex] = result.classification[1].value * 1000;

        voiceTotal = voiceTotal + voice[readIndex];
        musicTotal = musicTotal + music[readIndex];
        silentTotal = silentTotal + silent[readIndex];

        readIndex++;

        if (readIndex >= numReadings)
        {
          readIndex = 0;
        }

        voiceAverage = voiceTotal / numReadings;
        musicAverage = musicTotal / numReadings;
        silentAverage = silentTotal / numReadings;

        if ((voiceAverage > musicAverage) && (voiceAverage > silentAverage))
        {
          currentMicMode = 1;
          Serial.println("Voice");
        }
        else if ((musicAverage > voiceAverage) && (musicAverage > silentAverage))
        {
          currentMicMode = 2;
          Serial.println("Music");
        }
        else if ((silentAverage > musicAverage) && (silentAverage > voiceAverage))
        {
          currentMicMode = 3;
          Serial.println("Silent");
        }
        else
        {
          currentMicMode = 0;
          Serial.println("Mixed");
        }
        Serial.print("Voice: ");
        Serial.println(voiceAverage);
        Serial.print("Music: ");
        Serial.println(musicAverage);
        Serial.print("Silent: ");
        Serial.println(silentAverage);
        prevMillisRecord = millis();
      }

#if EI_CLASSIFIER_HAS_ANOMALY == 1
      ei_printf("    anomaly score: % .3f\n", result.anomaly);
#endif
    }
  }
  else
  {

    delay(readingTime);

    bool m = microphone_inference_record();
    if (!m) {
      ei_printf("ERR: Failed to record audio...\n");
      return;
    }

    signal_t signal;
    signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
    signal.get_data = &microphone_audio_signal_get_data;
    ei_impulse_result_t result = { 0 };

    EI_IMPULSE_ERROR r = run_classifier(&signal, &result, debug_nn);
    if (r != EI_IMPULSE_OK) {
      ei_printf("ERR: Failed to run classifier ( % d)\n", r);
      return;
    }

    voiceTotal = voiceTotal - voice[readIndex];
    musicTotal = musicTotal - music[readIndex];
    silentTotal = silentTotal - silent[readIndex];

    voice[readIndex] = result.classification[2].value * 1000;
    music[readIndex] = result.classification[0].value * 1000;
    silent[readIndex] = result.classification[1].value * 1000;

    voiceTotal = voiceTotal + voice[readIndex];
    musicTotal = musicTotal + music[readIndex];
    silentTotal = silentTotal + silent[readIndex];

    readIndex++;

    if (readIndex >= numReadings)
    {
      readIndex = 0;
    }

    voiceAverage = voiceTotal / numReadings;
    musicAverage = musicTotal / numReadings;
    silentAverage = silentTotal / numReadings;

    if ((voiceAverage > musicAverage) && (voiceAverage > silentAverage))
    {
      currentMicMode = 1;
      Serial.println("Voice");
    }
    else if ((musicAverage > voiceAverage) && (musicAverage > silentAverage))
    {
      currentMicMode = 2;
      Serial.println("Music");
    }
    else if ((silentAverage > musicAverage) && (silentAverage > voiceAverage))
    {
      currentMicMode = 3;
      Serial.println("Silent");
    }
    else
    {
      currentMicMode = 0;
      Serial.println("Mixed");
    }

    Serial.print("Voice: ");
    Serial.println(voiceAverage);
    Serial.print("Music: ");
    Serial.println(musicAverage);
    Serial.print("Silent: ");
    Serial.println(silentAverage);

#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("    anomaly score: %.3f\n", result.anomaly);
#endif
  }
}

void ei_printf(const char *format, ...) {
  static char print_buf[1024] = { 0 };

  va_list args;
  va_start(args, format);
  int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
  va_end(args);

  if (r > 0) {
    Serial.write(print_buf);
  }
}

static void pdm_data_ready_inference_callback(void)
{
  int bytesAvailable = PDM.available();

  // read into the sample buffer
  int bytesRead = PDM.read((char *)&sampleBuffer[0], bytesAvailable);

  if (inference.buf_ready == 0) {
    for (int i = 0; i < bytesRead >> 1; i++) {
      inference.buffer[inference.buf_count++] = sampleBuffer[i];

      if (inference.buf_count >= inference.n_samples) {
        inference.buf_count = 0;
        inference.buf_ready = 1;
        break;
      }
    }
  }
}

static bool microphone_inference_start(uint32_t n_samples)
{
  inference.buffer = (int16_t *)malloc(n_samples * sizeof(int16_t));

  if (inference.buffer == NULL) {
    return false;
  }

  inference.buf_count  = 0;
  inference.n_samples  = n_samples;
  inference.buf_ready  = 0;

  // configure the data receive callback
  PDM.onReceive(&pdm_data_ready_inference_callback);

  // optionally set the gain, defaults to 20
  PDM.setGain(80);
  PDM.setBufferSize(4096);

  // initialize PDM with:
  // - one channel (mono mode)
  // - a 16 kHz sample rate
  if (!PDM.begin(1, EI_CLASSIFIER_FREQUENCY)) {
    ei_printf("Failed to start PDM!");
    microphone_inference_end();

    return false;
  }

  return true;
}

static bool microphone_inference_record(void)
{
  inference.buf_ready = 0;
  inference.buf_count = 0;

  while (inference.buf_ready == 0) {
    delay(10);
  }

  return true;
}

static int microphone_audio_signal_get_data(size_t offset, size_t length, float * out_ptr)
{
  numpy::int16_to_float(&inference.buffer[offset], out_ptr, length);

  return 0;
}

static void microphone_inference_end(void)
{
  PDM.end();
  free(inference.buffer);
}

void updateBLE()
{
  if ((currentMicMode != prevMicMode)||(currMillis-prevMillisBLEupdate>BLEupdatePeriod))
  {
    micServiceChar.writeValue(currentMicMode);
    prevMicMode = currentMicMode;
    prevMillisBLEupdate=millis();
    Serial.println("Sending to BLE.");
  }
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_MICROPHONE
#error "Invalid model for current sensor."
#endif
