#ifndef CONFIG_UTILS_H_
#define CONFIG_UTILS_H_

#include "mongoose_glue.h"
#include "LittleFS.h"
#include "ArduinoJson-v7.3.0.h"

// File system variables
LittleFS_Program aioFS;
#define NF 10
static File files[NF + 1];
static int fsInit = 0;
// End

// Configuration system variables
struct Config
{
  char fversion[25];
};
const char *filename = "/config.txt";
// Config config;

settings s_config;
// End

// Initialize LittleFS disk
void fileInit()
{
  aioFS.begin(1 * 1024 * 1024);
  Serial.print("\r\nLittleFS: initialized");
  Serial.printf("\r\nTotal Size: %llu bytes, Used: %llu\r\n", aioFS.totalSize(), aioFS.usedSize());
  for (int i = 0; i <= NF; i++)
  {
    files[i] = 0;
  }
  fsInit = 1;
}

// Load configuration - Should load default config if run for the first time
void loadConfig(const char *filename, settings &s_config)
{
  Serial.println(F("Loading configuration..."));
  // Serial.printf("%s,%d,%d,%d,%d,%o\r\n", s_config.fversion,s_config.bd_ip1,s_config.bd_ip2, s_config.bd_ip3, s_config.gps_type,s_config.gps_pass);
  File file = aioFS.open(filename);
  JsonDocument doc;

  DeserializationError error = deserializeJson(doc, file);
  if (error)
    Serial.println(F("Failed to read config file, using default configuration"));
  // Populate the configuation array from the JSON doc | from the default values
  strlcpy(s_config.fversion,                     // <- destination
          doc["fversion"] | "AiO v5.0a Web GUI", // <- source Note: value after the | is the default if "fversion" is empty in the JSON document.
          sizeof(s_config.fversion));            // <- destination's capacity
  s_config.bd_ip1 = doc["bd_ip1"] | 192;
  s_config.bd_ip2 = doc["bd_ip2"] | 168;
  s_config.bd_ip3 = doc["bd_ip3"] | 5;
  s_config.gps_type = doc["gps_type"] | 2;
  s_config.gps_pass = doc["gps_pass"] | false;
  // End
  glue_set_settings(&s_config);
  file.close();
}

// Saves the configuration to a file
extern "C" void saveConfig(const char *filename, const settings &s_config)
{
  Serial.println(F("Saving configuration..."));
  // glue_update_state();
  glue_get_settings(&s_config);
  // Delete existing file, otherwise the configuration is appended to the file
  aioFS.remove(filename);

  File file = aioFS.open(filename, FILE_WRITE);
  if (!file)
  {
    Serial.println(F("Failed to create file"));
    return;
  }

  JsonDocument doc;

  doc["fversion"] = s_config.fversion;
  doc["bd_ip1"] = s_config.bd_ip1;
  doc["bd_ip2"] = s_config.bd_ip2;
  doc["bd_ip3"] = s_config.bd_ip3;
  doc["gps_type"] = s_config.gps_type;
  doc["gps_pass"] = s_config.gps_pass;

  if (serializeJson(doc, file) == 0)
  {
    Serial.println(F("Failed to write to file"));
  }

  file.close();
  Serial.printf("s_config: %s,%d,%d,%d,%d,%o\r\n", s_config.fversion, s_config.bd_ip1, s_config.bd_ip2, s_config.bd_ip3, s_config.gps_type, s_config.gps_pass);
  Serial.println(F("Saved configuration..."));
}

// Prints the JSON content of a file to the Serial port
void printFile(const char *filename)
{
  // Dump config file
  Serial.println(F("Print config file..."));
  File file = aioFS.open(filename);
  if (!file)
  {
    Serial.println(F("Failed to read file"));
    return;
  }

  while (file.available())
  {
    Serial.print((char)file.read());
  }
  Serial.println();

  file.close();
}

#endif