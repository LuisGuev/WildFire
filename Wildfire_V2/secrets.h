/*
Contains sensitive SIM card info and header files.
*/

// GSM settings
#define SECRET_PINNUMBER ""
#define SECRET_GPRS_APN "soracom.io"  // replace your GPRS APN
#define SECRET_GPRS_LOGIN "sora"      // replace with your GPRS login
#define SECRET_GPRS_PASSWORD "sora"   // replace with your GPRS password

// AWS settings
#define S3_SERVER "beam.soracom.io"
#define HTTP_PORT 18080


#define IOT_SERVER "beam.soracom.io"
#define MQTT_PORT 1883
#define TOPIC "arduino/outgoing"

/*
S3 Bucket
  name: device-videos
  ARN:  arn:aws:s3:::device-videos

IAM Role
  external id: External-ID-bH2q9KoalHPGi8w8
  ARN: arn:aws:iam::610134639275:role/beam-s3-role-2
*/