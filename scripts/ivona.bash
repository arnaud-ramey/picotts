#!/bin/bash
# Simple voice generation using IVONA web service.
# Synopsis:
# ivona.bash CREDENTIALFILE SENTENCE LANG VOICENAME RATE VOLUME
#
# CREDENTIALFILE: a text file containing two lines,
#   the first being the access key, the second the secret key
# SENTENCE: the SENTENCE to synthetise, for instance "Hello world"
# LANG: The case-sensitive language code of the TTS voice.
#   Language code according to the BCP47 recommendation (http://tools.ietf.org/html/bcp47)
#   with lowercase language code and uppercase country/region codes.
# VOICENAME: listed at http://www.ivona.com/us/about-us/voice-portfolio/
# RATE: possible values: "x-slow", "slow" "medium", "fast", "x-fast"
# VOLUME: possible values: "silent", "x-soft", "soft", "medium", "loud", "x-loud"
#
# Based on https://github.com/bblocks/aws-api-v4/blob/master/post-simple.bash
# and http://developer.ivona.com/en/speechcloud/dev_guide_request_signing.html
#
# Useful links:
# http://developer.ivona.com/en/speechcloud/api_ref_data_types.html#DataTypes_Data
# http://developer.ivona.com/en/speechcloud/dev_guide_using_http_get_for_api_requests.html
if [ $# -lt 1 ]
then
  echo "Simple voice generation using IVONA web service."
  echo "Synopsis: ivona.bash CREDENTIALFILE SENTENCE LANG VOICENAME RATE VOLUME"
  exit -1
fi
CREDENTIALFILE=$1
SENTENCE=${2:-"Default sentence. Time is `date '+%H:%M:%S'`"}
LANG=${3:-"en-GB"}
VOICENAME=${4:-"Amy"}
RATE=${5:-"medium"}
VOLUME=${6:-"medium"}

# read credentials
if [ ! -f $CREDENTIALFILE ]; then
  echo "Could not open CREDENTIALFILE '$CREDENTIALFILE'"
  exit -1
fi
mapfile -t lines < "$CREDENTIALFILE"
access_key="${lines[0]}"
secret_key="${lines[1]}"
mp3outputfile="/tmp/EttsIvona.mp3"
region='eu-west-1'
service='tts'
host="$service.$region.ivonacloud.com"
payload="{
\"Input\":
  {\"Data\":\"$SENTENCE\"},
\"Parameters\":
  {\"Rate\":\"$RATE\",\"Volume\":\"$VOLUME\"},
\"Voice\":
  {\"Name\":\"$VOICENAME\",\"Language\":\"$LANG\"}
}"
# convert encoding UTF8->ISO
payload=`echo "$payload" | iconv -f utf-8 -t iso-8859-1`


x_amz_date_long=$(date -u '+%Y%m%dT%H%M%SZ')
x_amz_date_short="${x_amz_date_long/T*}"
content_type='application/json'

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Task 1: Create a canonical request
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
http_request_method='POST'
canonical_uri='/CreateSpeech'
canonical_query_string=''

hashed_payload=$(printf "$payload" | sha256sum)
hashed_payload="${hashed_payload%% *}"

canonical_headers="content-type:$content_type
host:$host
x-amz-content-sha256:$hashed_payload
x-amz-date:$x_amz_date_long"

signed_headers='content-type;host;x-amz-content-sha256;x-amz-date'

canonical_request="$http_request_method
$canonical_uri
$canonical_query_string
$canonical_headers

$signed_headers
$hashed_payload"

hashed_canonical_request=$(printf "$canonical_request" | sha256sum)
hashed_canonical_request="${hashed_canonical_request%% *}"


#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Task 2: Create a string to sign
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
string_to_sign="AWS4-HMAC-SHA256
$x_amz_date_long
$x_amz_date_short/$region/$service/aws4_request
$hashed_canonical_request"


#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Task 3: Calculate the signature
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
hex_secret=$(printf "AWS4$secret_key" | xxd -p -c 256)
digest=$(printf "$x_amz_date_short" | openssl dgst -sha256 -mac HMAC -macopt hexkey:$hex_secret)
digest="${digest#* }"
digest=$(printf "$region" | openssl dgst -sha256 -mac HMAC -macopt hexkey:$digest)
digest="${digest#* }"
digest=$(printf "$service" | openssl dgst -sha256 -mac HMAC -macopt hexkey:$digest)
digest="${digest#* }"
digest=$(printf "aws4_request" | openssl dgst -sha256 -mac HMAC -macopt hexkey:$digest)
signing_key="${digest#* }"

signature=$(printf "$string_to_sign" | openssl dgst -binary -hex -sha256 -mac HMAC -macopt hexkey:$signing_key)
signature="${signature#* }"


#~~~~~~~~
# Try it
#~~~~~~~~
read -r -d '' curl_params<<END
url = "https://$host${canonical_uri}"
-X POST
-H "Host: $host"
-H "Content-Type: $content_type"
-H "X-Amz-Date: $x_amz_date_long"
-H "Authorization: AWS4-HMAC-SHA256 Credential=$access_key/$x_amz_date_short/$region/$service/aws4_request, SignedHeaders=$signed_headers, Signature=$signature"
-H "x-amz-content-sha256: $hashed_payload"
-H "Content-Length: ${#payload}"
END

#~ echo "*** canonical_request ***"; echo "${canonical_request}" ; echo "*** eof ***"; echo
#~ echo "*** string_to_sign ***"; echo "${string_to_sign}" ; echo "*** eof ***"; echo
#~ echo "*** signature ***"; echo "${signature}" ; echo "*** eof ***"; echo
#~ echo "*** curl_params ***"; echo "${curl_params}" ; echo "*** eof ***"; echo
#~ echo "*** payload ***"; echo "${payload}" ; echo "*** eof ***"; echo

echo "Trying to download file to $mp3outputfile..."
#~ curl -vvv --output $mp3outputfile --data-ascii "$payload" --config - < <(printf "$curl_params")
curl --output $mp3outputfile --data-ascii "$payload" --config - < <(printf "$curl_params")
# Will exit with status of last command = curl
