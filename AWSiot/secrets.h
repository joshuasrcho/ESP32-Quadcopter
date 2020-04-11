#include <pgmspace.h>

#define SECRET
#define THINGNAME "QuadcopterESP32Cam"

const char WIFI_SSID[] = "HappyJoy";
const char WIFI_PASSWORD[] = "6608ohla";
const char AWS_IOT_ENDPOINT[] = "a2zylysdeluzll-ats.iot.us-west-1.amazonaws.com";

// Amazon Root CA 1
static const char AWS_CERT_CA[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI
U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs
N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv
o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU
5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy
rqXRfboQnoZsG4q5WTP468SQvvG5
-----END CERTIFICATE-----

)EOF";

// Device Certificate
static const char AWS_CERT_CRT[] PROGMEM = R"KEY(
-----BEGIN CERTIFICATE-----
MIIDWTCCAkGgAwIBAgIUJT5YKkEbWVB3ntBMbnSi+vUrM5owDQYJKoZIhvcNAQEL
BQAwTTFLMEkGA1UECwxCQW1hem9uIFdlYiBTZXJ2aWNlcyBPPUFtYXpvbi5jb20g
SW5jLiBMPVNlYXR0bGUgU1Q9V2FzaGluZ3RvbiBDPVVTMB4XDTIwMDQxMDIzMzE1
OVoXDTQ5MTIzMTIzNTk1OVowHjEcMBoGA1UEAwwTQVdTIElvVCBDZXJ0aWZpY2F0
ZTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAN05QyiJCvdIdkstcisQ
lA7fgOfix+0jNi+rnekKTT9HgVH1douhCAlenNWdyXhFKh7l6oXit9RGdglbUGes
4bFjbtivCLjbZrrJa29NMUCss5J5OKIeaKa/sLetadGMSpnjOpRyp7+C8ufgdFWz
VGtKJw+JQcKnsWBluO6xm9ySLFoW7+GIEjOFzHz2KyJbYDFEAz2VqbPK997xXmRZ
Rdzyn7iHWyFYz2y83f+7n25OXrwat0GGHqJTjb2KAKCskSR/oYJZ49uBFRVN8beq
1B/4AQJi1JuuWelJ1cNJL3Xxtp4ByvUUOZoLPbQWboWd3d+I4jwwn5ImBQUlfl4C
nPMCAwEAAaNgMF4wHwYDVR0jBBgwFoAU5plSpJKJxBIiXYYbATAWG+r89qEwHQYD
VR0OBBYEFHeC/fWeDWf4XmPeZfELQ6XviYeXMAwGA1UdEwEB/wQCMAAwDgYDVR0P
AQH/BAQDAgeAMA0GCSqGSIb3DQEBCwUAA4IBAQBJ8U1JYEoBOOt3WuRKl2roILhP
E/GcDIMHBp67OWsTn5Wum8TM5FxGAA7+fh3WGFU49cuxjx4ZrLUel7Gttl/9OAEc
A1MPzbOA7u0JalJI+rdmlk/JOoUHaTmW0HxpfMUyadqj9E9mX0OHQUvwT1OmGH1C
6set+fZXwEKVWyZwdnA0iJrL85KJ5c9NlQITVjDBLO1ayjrVRbbWvaLW/xVpPwV4
On0WhkdbVI6S5v9ze00hmgCBdO7+rJbU4cfZdRgPxGxia2tTUyrZzHblHStGubK2
8fVHu9GC6YcWjm1jJdzCAZ4onr9Uoi0mt1sQEusWR3rekS+7EOJ3pCjEjClq
-----END CERTIFICATE-----
)KEY";

// Device Private Key
static const char AWS_CERT_PRIVATE[] PROGMEM = R"KEY(
-----BEGIN RSA PRIVATE KEY-----
MIIEpQIBAAKCAQEA3TlDKIkK90h2Sy1yKxCUDt+A5+LH7SM2L6ud6QpNP0eBUfV2
i6EICV6c1Z3JeEUqHuXqheK31EZ2CVtQZ6zhsWNu2K8IuNtmuslrb00xQKyzknk4
oh5opr+wt61p0YxKmeM6lHKnv4Ly5+B0VbNUa0onD4lBwqexYGW47rGb3JIsWhbv
4YgSM4XMfPYrIltgMUQDPZWps8r33vFeZFlF3PKfuIdbIVjPbLzd/7ufbk5evBq3
QYYeolONvYoAoKyRJH+hglnj24EVFU3xt6rUH/gBAmLUm65Z6UnVw0kvdfG2ngHK
9RQ5mgs9tBZuhZ3d34jiPDCfkiYFBSV+XgKc8wIDAQABAoIBAQC+ywYazabBgqNT
TwyqC5BGC4kXdMVMxhOTpYV5WUMqEjd8QiFNZ0KRIiJNMdpyeeRL5asQcWpuD575
juGXqMi1vrX492ykUcUVz2VzBrY1uzzcVOP9HhDPz39PyqX67e3hLtJ0+TWnu/5W
8YQj7ZgfOELb6Vrn0MzwAIswgcmUvnnyIaM7RlHvvQsuuc9PsfgLCjKSuS1YBFup
d+UUh3/BBh53VfliJ8HrTs+QT8EhDqKE93KgOHfV4t66+DZa8e88zYWFMgZYM21K
1rgwt0q6raqRqPIZAjUfvD5im50EPo4SimrumYSAcHNsSlo2+hkkdWvsTW+5fAgd
x9iLoZkRAoGBAPbVwRp6RlDtFCSDGveu+TSo7FTcHNRN+Jp2jd3H4MT8NTExAPij
yIQPniylbGlouLxpzLpGyQ1WeSAi0KNF1wxkjf0ALxrmdzkq6w9vAKsY5VjfUXyZ
W0JFThs+Y0NlrbiVVBdqodrcahN3t+981tQzNnoNK02e1yXgHpTUWGUNAoGBAOVw
EH6JvDgmB0wNXRSXRrTDWRPy2goH4yR8Rh+aSPar1I9p9x9gRcneap3TqLqWjniE
yZCkanj+2Ct4N17O2AuG07+UsZOMNyvrF7+slyiAm5+dtj6OHNvmidPVZCl2T4Wy
Z2a9Ka4qR+5diHPZpSZSiERAC2FM3qq0z7xm6on/AoGBAM7MeVNwnBbS+ew5N0/J
9ZnL6OyT2Ud5582pg+QKwOLolrwW3URzG9ygpRmsom6RwSyBy3dJNMqK50IoSPpx
2eDVIiyUieKUsdBgh71Pd5Thb80MlF26sj2MiQrEMy/VMT2TRnez69TMk8app53q
zqXMHlkVTXFjocdmxNhWK5TFAoGBALZGFkSA5e/xfkCUKUtvdvOqcf0gJMptQ627
sPXwW2DkV8TrmTDrCeZ69VaXTWeWrm7eh0eK5y9ivpeiNNL0Hrzhix2bYovmi/ov
Fbnnwlqh+YCQCbWerxygBQUudRlfbYO4nN8JDrDX1DdxRTNMSCCu1qTqAGB3cjpJ
1PNHC7M/AoGATpioKxJOd70rNXV2KvuQgIo7KRqZmGw+gb6K3OiFyDht4k/8N+Yo
QPwfZoJ+Sq+pmqm88QYfMsr40moyVwYUS1BK+vnLRGr4h+YC5sa2NddTCCgsf9vR
WZ9CqC8HI0Egpw0R0G/526sRRVH6bedfe8ZEZlWrpqLPQC72ibB/AZ0=
-----END RSA PRIVATE KEY-----
)KEY";
