/*	Node that performs text-to-speech conversion by using Google's
	speech synthesiser.
	Alexander Reiter, Institute for Robotics (ROBIN), JKU Linz
	November-December 2013
	
	This node sends request to the soundplay_node to play audio files
	that contain synthesised speech. It checks with a dictonary if the 
	to-be-synthesised text has been handled before. If so, the existing
	audio file is used, otherwise, synthesised speech is downloaded from
	Google. The filename is a 50 character string and is chosen randomly.
	To determine whether an audio file that corresponds to the
	text already exists, the node accesses a json dictionary that 
	contains pre-downloaded files with synthesised speech. It holds 
	strings as keys and filenames as values.
	
	
	PARAMETERS:
	audioPath: path to folder with audio files (does not need to end with '/')
	jsonPath: path to json file with dictionary
	language: language code for speech synthesis (en, de, etc.)
	volume: volume value for the reproduced speech (0.0 - 1.0)

	
	References:
	Google TTS API
	http://elinux.org/RPi_Text_to_Speech_%28Speech_Synthesis%29#Google_Text_to_Speech
	Jansson API
	https://jansson.readthedocs.org/en/2.5/apiref.html
*/

#include <iostream>
#include <sstream>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sound_play/SoundRequest.h"
#include <ctype.h>
#include <stdio.h>
#include <curl/curl.h>
#include <jansson.h>
#include <time.h>
#include <iconv.h>

using namespace std;

void stringCallback(const std_msgs::String msg);
void createFilename();

ros::Publisher soundPub;	// publisher for sound_play requests

// ROS parameters variables
string audioPath;	// path to audio folder
string jsonPath;	// path to json file with audio references
string language;	// language code
float volume;		// Volume for the sound_play node.
bool mplayer;		// using mplayer for audio output

json_t* root;	// root node of json structure

// curl variables
CURL *curl;
CURLcode res;
char filename[51];

/* UTF-8 to ISO-8859-1/ISO-8859-15 mapper.
 * Return 0..255 for valid ISO-8859-15 code points, 256 otherwise.
*/
static inline unsigned int to_latin9(const unsigned int code)
{
    /* Code points 0 to U+00FF are the same in both. */
    if (code < 256U)
        return code;
    switch (code) {
    case 0x0152U: return 188U; /* U+0152 = 0xBC: OE ligature */
    case 0x0153U: return 189U; /* U+0153 = 0xBD: oe ligature */
    case 0x0160U: return 166U; /* U+0160 = 0xA6: S with caron */
    case 0x0161U: return 168U; /* U+0161 = 0xA8: s with caron */
    case 0x0178U: return 190U; /* U+0178 = 0xBE: Y with diaresis */
    case 0x017DU: return 180U; /* U+017D = 0xB4: Z with caron */
    case 0x017EU: return 184U; /* U+017E = 0xB8: z with caron */
    case 0x20ACU: return 164U; /* U+20AC = 0xA4: Euro */
    default:      return 256U;
    }
}

/* Convert an UTF-8 string to ISO-8859-15.
 * All invalid sequences are ignored.
 * Note: output == input is allowed,
 * but   input < output < input + length
 * is not.
 * Output has to have room for (length+1) chars, including the trailing NUL byte.
*/
size_t utf8_to_latin9(char *const output, const char *const input, const size_t length)
{
    unsigned char             *out = (unsigned char *)output;
    const unsigned char       *in  = (const unsigned char *)input;
    const unsigned char *const end = (const unsigned char *)input + length;
    unsigned int               c;

    while (in < end)
        if (*in < 128)
            *(out++) = *(in++); /* Valid codepoint */
        else
        if (*in < 192)
            in++;               /* 10000000 .. 10111111 are invalid */
        else
        if (*in < 224) {        /* 110xxxxx 10xxxxxx */
            if (in + 1 >= end)
                break;
            if ((in[1] & 192U) == 128U) {
                c = to_latin9( (((unsigned int)(in[0] & 0x1FU)) << 6U)
                             |  ((unsigned int)(in[1] & 0x3FU)) );
                if (c < 256)
                    *(out++) = c;
            }
            in += 2;

        } else
        if (*in < 240) {        /* 1110xxxx 10xxxxxx 10xxxxxx */
            if (in + 2 >= end)
                break;
            if ((in[1] & 192U) == 128U &&
                (in[2] & 192U) == 128U) {
                c = to_latin9( (((unsigned int)(in[0] & 0x0FU)) << 12U)
                             | (((unsigned int)(in[1] & 0x3FU)) << 6U)
                             |  ((unsigned int)(in[2] & 0x3FU)) );
                if (c < 256)
                    *(out++) = c;
            }
            in += 3;

        } else
        if (*in < 248) {        /* 11110xxx 10xxxxxx 10xxxxxx 10xxxxxx */
            if (in + 3 >= end)
                break;
            if ((in[1] & 192U) == 128U &&
                (in[2] & 192U) == 128U &&
                (in[3] & 192U) == 128U) {
                c = to_latin9( (((unsigned int)(in[0] & 0x07U)) << 18U)
                             | (((unsigned int)(in[1] & 0x3FU)) << 12U)
                             | (((unsigned int)(in[2] & 0x3FU)) << 6U)
                             |  ((unsigned int)(in[3] & 0x3FU)) );
                if (c < 256)
                    *(out++) = c;
            }
            in += 4;

        } else
        if (*in < 252) {        /* 111110xx 10xxxxxx 10xxxxxx 10xxxxxx 10xxxxxx */
            if (in + 4 >= end)
                break;
            if ((in[1] & 192U) == 128U &&
                (in[2] & 192U) == 128U &&
                (in[3] & 192U) == 128U &&
                (in[4] & 192U) == 128U) {
                c = to_latin9( (((unsigned int)(in[0] & 0x03U)) << 24U)
                             | (((unsigned int)(in[1] & 0x3FU)) << 18U)
                             | (((unsigned int)(in[2] & 0x3FU)) << 12U)
                             | (((unsigned int)(in[3] & 0x3FU)) << 6U)
                             |  ((unsigned int)(in[4] & 0x3FU)) );
                if (c < 256)
                    *(out++) = c;
            }
            in += 5;

        } else
        if (*in < 254) {        /* 1111110x 10xxxxxx 10xxxxxx 10xxxxxx 10xxxxxx 10xxxxxx */
            if (in + 5 >= end)
                break;
            if ((in[1] & 192U) == 128U &&
                (in[2] & 192U) == 128U &&
                (in[3] & 192U) == 128U &&
                (in[4] & 192U) == 128U &&
                (in[5] & 192U) == 128U) {
                c = to_latin9( (((unsigned int)(in[0] & 0x01U)) << 30U)
                             | (((unsigned int)(in[1] & 0x3FU)) << 24U)
                             | (((unsigned int)(in[2] & 0x3FU)) << 18U)
                             | (((unsigned int)(in[3] & 0x3FU)) << 12U)
                             | (((unsigned int)(in[4] & 0x3FU)) << 6U)
                             |  ((unsigned int)(in[5] & 0x3FU)) );
                if (c < 256)
                    *(out++) = c;
            }
            in += 6;

        } else
            in++;               /* 11111110 and 11111111 are invalid */

    /* Terminate the output string. */
    *out = '\0';

    return (size_t)(out - (unsigned char *)output);
}

/*
std::string UTF8toISO8859_1(const char * in) {
    std::string out;
    if (in == NULL)
        return out;

    unsigned int codepoint;
    while (*in != 0) {
        unsigned char ch = static_cast<unsigned char>(*in);
        if (ch <= 0x7f)
            codepoint = ch;
        else if (ch <= 0xbf)
            codepoint = (codepoint << 6) | (ch & 0x3f);
        else if (ch <= 0xdf)
            codepoint = ch & 0x1f;
        else if (ch <= 0xef)
            codepoint = ch & 0x0f;
        else
            codepoint = ch & 0x07;
        ++in;

        if (((*in & 0xc0) != 0x80) && (codepoint <= 0x10ffff)) {
            // a valid codepoint has been decoded; convert it to ISO-8859-15               
            char outc;
            if (codepoint <= 255) {
                // codepoints up to 255 can be directly converted wit a few exceptions
                if (codepoint != 0xa4 && codepoint != 0xa6 && codepoint != 0xa8
                        && codepoint != 0xb4 && codepoint != 0xb8 && codepoint != 0xbc
                        && codepoint != 0xbd && codepoint != 0xbe) {
                    outc = static_cast<char>(codepoint);
                }
                else {
                    outc = '?';
                }
            }
            else {
                // With a few exceptions, codepoints above 255 cannot be converted
                if (codepoint == 0x20AC) {
                    outc = 0xa4;
                }
                else if (codepoint == 0x0160) {
                    outc = 0xa6;
                }
                else if (codepoint == 0x0161) {
                    outc = 0xa8;
                }
                else if (codepoint == 0x017d) {
                    outc = 0xb4;
                }
                else if (codepoint == 0x017e) {
                    outc = 0xb8;
                }
                else if (codepoint == 0x0152) {
                    outc = 0xbc;
                }
                else if (codepoint == 0x0153) {
                    outc = 0xbd;
                }
                else if (codepoint == 0x0178) {
                    outc = 0xbe;
                }
                else {
                    outc = '?';
                }
            }
            out.append(1, outc);
        }
    }
    return out;
}
*/
 
int main(int argc, char** argv) {
  
  // init ROS
  ros::init(argc, argv, "speech_database");
  ros::NodeHandle n("~");
  ros::spinOnce();
  ros::Subscriber sub;
  sub = n.subscribe("/speech", 10, stringCallback);
  soundPub = n.advertise<sound_play::SoundRequest>("/robotsound", 10);
  
  ros::Rate loop_rate(10);
  if(!n.getParam("audioPath", audioPath)) {
	  ROS_ERROR("Parameter audioPath not found");
	  ros::shutdown();
  }
  // check if audioPath ends with '/'
  if(audioPath.find_last_of("/") != audioPath.length()-1) {
    audioPath.append("/");
  }
  if(!n.getParam("language", language)) {
	  ROS_ERROR("Parameter language not found");
	  ros::shutdown();
  }
  if(!n.getParam("jsonPath", jsonPath)) {
	  ROS_ERROR("Parameter jsonPath not found");
	  ros::shutdown();
  }
  if(!n.getParam("mplayer", mplayer)) {
	  ROS_ERROR("Parameter mplayer not found");
	  ros::shutdown();
  }
  if (!n.hasParam("volume")) {
      ROS_WARN("Volume parameter not found. Setting default volume to 0.5.");
  }
  n.param<float>("volume", volume, 0.5f);

  // prepare rand
  srand(time(NULL));
  srand(time(NULL) + rand());
  
  // init json file
  json_error_t err;
  root = json_load_file(jsonPath.c_str(), 0, &err);
  if(!root) {
	  ROS_DEBUG("json file %s does not exist, a new file will be created", jsonPath.c_str());
	  root = json_object();
  } else {
	  ROS_DEBUG("json file %s successfully opened", jsonPath.c_str());
  }
  
  // init curl
  curl = curl_easy_init();
  
  curl_easy_setopt(curl, CURLOPT_USERAGENT, "Mozilla");
 
  while (curl && ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  curl_easy_cleanup(curl);
 
  return 0;
}

/* Callback function that checks if the current json structure already
 * contains an entry that corresponds to the string from the message. If
 * no entry exists, it a new a new speech file is downloaded and stored
 * using curl and a new json entry is created.
 * The path of the audio file that corresponds to the string in message
 * is stored in a SoundRequest message and advertised on the robotsound
 * topic.
 * Since Google TTS only supports string with a length of up to 100 
 * characters, a warning is thrown when a string is to be synthesised
 * the exceeds the maximum length.
 * 
 * INPUT:	const std_msgs::String msg: message with string to be translated
 * OUTPUT:	none
 */
void stringCallback(const std_msgs::String msg) {
	string data = msg.data;
	if(data.length() >  100) {
		ROS_WARN("The exceeds the maximum length of 100 characters and will not be synthesised.");
		return;
	}

	cout << data << endl;
	printf("%s\n", data);

	//
    iconv_t cd = iconv_open("UTF-8//TRANSLIT", "ISO_8859-1");
    if (cd == (iconv_t) -1) {
        perror("iconv_open failed!");
        //return 1;
    }

    char charstr[] = "ent\xE3o conta-me l\xE1 como \xE9 que te chamas";
    char *in_buf = &charstr[0];
	size_t in_left = sizeof(charstr) - 1;

	printf("hardcode string: %d\n", strlen(charstr));
	for(int i = 0; i < strlen(charstr); i++) {
		printf("(%d %c) ", charstr[i], charstr[i]);
	}
	printf("\n");

    char output[255];
    char *out_buf = &output[0];
    size_t out_left = sizeof(output) - 1;

    do {
        if (iconv(cd, &in_buf, &in_left, &out_buf, &out_left) == (size_t) -1) {
            perror("iconv failed!");
            //return 1;
        }
    } while (in_left > 0 && out_left > 0);
    *out_buf = 0;

  	printf("%s -> %s\n", charstr, output);

/*
	char charstr[255];
	//std::string sample = data;
	std::copy( data.begin(), data.end(), charstr );
	charstr[data.length()] = 0;

	char *in_buf = &charstr[0];
    size_t in_left = sizeof(charstr) - 1;

    char output[255];
    char *out_buf = &output[0];
    size_t out_left = sizeof(output) - 1;

    do {
        if (iconv(cd, &in_buf, &in_left, &out_buf, &out_left) == (size_t) -1) {
            perror("iconv failed!");
            //return 1;
        }
    } while (in_left > 0 && out_left > 0);
    *out_buf = 0;

  	printf("%s -> %s\n", charstr, output);
*/

	//std::string sample = data;
	std::copy( data.begin(), data.end(), charstr );
	charstr[data.length()] = 0;
/*
	in_buf = &charstr[0];
    in_left = sizeof(charstr) - 1;

    out_buf = &output[0];
    out_left = sizeof(output) - 1;

    do {
        if (iconv(cd, &in_buf, &in_left, &out_buf, &out_left) == (size_t) -1) {
            perror("iconv failed!");
            //return 1;
        }
    } while (in_left > 0 && out_left > 0);
    *out_buf = 0;

  	printf("%s -> %s\n", charstr, output);

	printf("dynamic string: %d\n", strlen(charstr));
	for(int i = 0; i < strlen(charstr); i++) {
		printf("(%d %c) ", charstr[i], charstr[i]);
	}
	printf("\n");
*/
    iconv_close(cd);

	printf("%s \n", charstr);

	int res2 = utf8_to_latin9(out_buf, charstr, strlen(charstr));

	printf("%d %s \n", res, out_buf);

/*
	std::copy( data.begin(), data.end(), charstr );
	charstr[data.length()] = 0;

	printf("dynamic string: %d\n", strlen(charstr));
	for(int i = 0; i < strlen(charstr); i++) {
		printf("(%d %c) ", charstr[i], charstr[i]);
	}
	printf("\n");

	std::string test1 =	UTF8toISO8859_1 (charstr);
	printf("nn\n");
    cout << test1 << endl;
	printf("%s\n", test1);
*/

	// convert string to lowercase
	for(int i = 0; i < data.length(); i++) {
		data[i] = (char) tolower((int) data[i]);
	}
	
	// check if entry for this string exists in json file
	json_t* entry = json_object_get(root, data.c_str());	// holds json_string object with audio file path
	if(!entry) {
		ROS_DEBUG("Entry does not exist, downloading new file");
		
		// prepare URL for curl
		stringstream urlStream;
        urlStream << "http://translate.google.com/translate_tts?ie=UTF-8&client=tw-ob&tl=";
		urlStream << language << "&q=";
		char* str = curl_easy_escape(curl, data.c_str(), 0);
		urlStream << str;
		curl_free(str);
		ROS_INFO(urlStream.str().c_str());
		
		stringstream filenameStream;
		FILE* file = fopen("/dev/null", "r");	// some existing file
		while(file) {	// generate new filenames until corresponding file does not exist
			curl_easy_setopt(curl, CURLOPT_URL, urlStream.str().c_str());
			filenameStream.str("");
			filenameStream.clear();
			filenameStream << audioPath;
			createFilename();
			filenameStream << filename;
			file = fopen(filenameStream.str().c_str(), "r");
		}
		// file does not exist, create it
		file = fopen(filenameStream.str().c_str(), "w");
		if(!file) {
			ROS_ERROR("Could not open file %s", filenameStream.str().c_str());
			fclose(file);
			return;
		}
		
		curl_easy_setopt(curl, CURLOPT_WRITEDATA, file) ;
		res = curl_easy_perform(curl);
		fclose(file);
		
		// check for curl errors
		if(res == CURLE_OK) {
			ROS_DEBUG("File downloaded and stored in %s", filenameStream.str().c_str());
		} else {
			ROS_ERROR("Download failed, curl_easy_perform() error: %s", curl_easy_strerror(res));
			return;
		}
		
		// add json object
		entry = json_string(filename);
		json_object_set(root, data.c_str(), entry);
		
		if(json_dump_file(root, jsonPath.c_str(), 0) == 0) {
			ROS_DEBUG("json file successfully written to %s", jsonPath.c_str());
		} else {
			ROS_ERROR("json file could not be written, all changes are lost");
		}
	}
	
	// create stop message
	sound_play::SoundRequest req;
	req.sound = req.ALL;
	req.command = req.PLAY_STOP;
	if(!mplayer) {
		soundPub.publish(req);
	}
	
	// create speech message
	req.sound = req.PLAY_FILE;
	req.command = req.PLAY_ONCE;
    req.volume = volume;
	string path_to_file = audioPath;
	path_to_file.append(json_string_value(entry));
	req.arg = path_to_file;
	
	if(mplayer) {
		string cmd_str = "mplayer ";
		cmd_str = cmd_str + path_to_file;
		system(cmd_str.c_str());
	} else {
		soundPub.publish(req);
	}
}

/* Function that writes a random sequence of (human readable) characters
 * to the global filename c string (0 terminated).
 * 
 * INPUT:	none
 * OUTPUT:	none
 */
void createFilename() {
	int i;
	char ch;
	for(i = 0; i < 50; i++) {
		int ran = rand() % 62;
		ch = (char) ran;
		if(ch < 10) {
			ch = (char) ch + '0'; // numbers
		} else if(ch < 36) {
			ch = (ch - 10) + 'A'; // capital letters
		} else {
			ch = (ch - 36) + 'a'; // small letters
		}
		filename[i] = ch;
	}
	filename[50] = 0;
}
