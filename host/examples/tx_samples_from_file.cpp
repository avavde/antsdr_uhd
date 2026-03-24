#include <uhd/types/tune_request.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/usrp_clock/multi_usrp_clock.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/error.h>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>

#include <map>
#include <string>
#include <chrono>
#include <complex>
#include <csignal>
#include <fstream>
#include <iostream>
#include <thread>
#include <algorithm>
#include <cstring>
#include <climits>

#ifndef MAX_PATH
#define MAX_PATH 4096
#endif

uhd::usrp::multi_usrp::sptr usrp;
uhd::usrp_clock::multi_usrp_clock::sptr usrpclock;

int eavg, econd;

char userAppdata_path[MAX_PATH] = { 0 }, userAppdata_path2[MAX_PATH] = { 0 };

void trace(const char *str, const char *fname = "tx_samples_from_file.log") {
    FILE *fp;
    const char *used_fname = fname;
    char logged_str[2048];

    fp = fopen(used_fname, "a");
    if (!fp) fp = fopen(used_fname, "w");
    if (fp != NULL) {
        auto currentTime = std::chrono::system_clock::now();
        std::time_t currentTimeT = std::chrono::system_clock::to_time_t(currentTime);
        auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime.time_since_epoch() % std::chrono::seconds(1));
        
        std::tm *ltm = std::localtime(&currentTimeT);
        sprintf(logged_str, "%02i:%02i:%02i %03lld %s", ltm->tm_hour, ltm->tm_min, ltm->tm_sec, (long long)milliseconds.count(), str);
        fprintf(fp, "%s\n", logged_str);
        fclose(fp);
    }
}



static bool stop_signal_called = false;
void sig_int_handler(int)
{
    stop_signal_called = true;
}


void get_gps_str() {
	//char bufstr[128];
	uhd::sensor_value_t gga_string = usrp->get_mboard_sensor("gps_gpgga"); //  usrpclock->get_sensor("gps_gpgga");
	//uhd::sensor_value_t rmc_string = usrpclock->get_sensor("gps_gprmc");
	//uhd::sensor_value_t servo_string = clock->get_sensor("gps_servo");
	//std::cout << "\nPrinting available NMEA strings:\n";
	//std::cout << boost::format("%s\n%s\n") % gga_string.to_pp_string()
	trace("GGA STRING:");
	trace(gga_string.to_pp_string().c_str());
}


template <typename samp_type>
void send_from_file(
	uhd::tx_streamer::sptr tx_stream, const std::string& file, size_t samps_per_buff)
{
	uhd::tx_metadata_t md;
	md.start_of_burst = false;
	md.end_of_burst = false;
	std::vector<samp_type> buff(samps_per_buff);
	char strbuf[64];
	long long elapsed_last=99999, avg_sum=99999999, cur_sum=0;
	int loop_n = 1;
	int avg_n = 0;


	if (file != "sinus") {

		trace("sending file: ");  trace(file.c_str());

		std::ifstream infile(file.c_str(), std::ifstream::binary);

		// loop until the entire file has been read

		while (not md.end_of_burst and not stop_signal_called) {
			infile.read((char*)&buff.front(), buff.size() * sizeof(samp_type));
			size_t num_tx_samps = size_t(infile.gcount() / sizeof(samp_type));

			md.end_of_burst = infile.eof();

			trace("[start] tx_stream->send");
			auto tm_real_send_start = std::chrono::high_resolution_clock::now();
			const size_t samples_sent = tx_stream->send(&buff.front(), num_tx_samps, md);
			long long elapsed_from_prev = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - tm_real_send_start).count();
			sprintf(strbuf, "[end] tm=%lld %lld %s", elapsed_from_prev, avg_sum, (elapsed_from_prev > (avg_sum+econd)/*elapsed_from_prev>(elapsed_last*2) && (elapsed_last>400)*/) ? "ERROR": "");
			trace(strbuf);
			elapsed_last = elapsed_from_prev;

			cur_sum += elapsed_from_prev;

			if ((loop_n % eavg)==0) {
				avg_sum = cur_sum / eavg;
				cur_sum = 0;
			}
			
			strcpy(strbuf, "None");
			uhd_get_last_error(strbuf, 64); // get last possible error
			if (strlen(strbuf)>5) trace(strbuf);

			if (samples_sent != num_tx_samps) {
				/*UHD_LOG_ERROR*/ std::cerr <<  /*"TX-STREAM",*/
					"The tx_stream timed out sending " << num_tx_samps << " samples ("
					<< samples_sent << " sent)." << std::endl;
				return;
			}

			if((loop_n % 1000) == 0) get_gps_str();
			loop_n++;
		}

		infile.close();
	




	} else {  // SINUS:

		trace("sending sinus:");

		// fill sinus:
		for (int i = 0; i < samps_per_buff; i+=4) {
			buff[i] = (samp_type)1.0;
			buff[i+1] = (sizeof(samp_type)==sizeof(short)) ? (samp_type)6000 : (samp_type)0.1;

			buff[i] = (samp_type)0.0;
			buff[i + 1] = (sizeof(samp_type) == sizeof(short)) ? (samp_type)3000 : (samp_type)0.2;
		}


		while (not md.end_of_burst and not stop_signal_called) {
			size_t num_tx_samps = samps_per_buff;

			md.end_of_burst = false; // infile.eof();

			trace("[start] tx_stream->send");
			auto tm_real_send_start = std::chrono::high_resolution_clock::now();
			const size_t samples_sent = tx_stream->send(&buff.front(), num_tx_samps, md);
			long long elapsed_from_prev = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - tm_real_send_start).count();
			sprintf(strbuf, "[end] tm=%lld %lld %s", elapsed_from_prev, avg_sum, (elapsed_from_prev > (avg_sum + econd)/*elapsed_from_prev>(elapsed_last*2) && (elapsed_last>400)*/) ? "ERROR" : "");
			trace(strbuf);
			elapsed_last = elapsed_from_prev;

			cur_sum += elapsed_from_prev;

			if ((loop_n % eavg) == 0) {
				avg_sum = cur_sum / eavg;
				cur_sum = 0;
			}

			strcpy(strbuf, "None");
			uhd_get_last_error(strbuf, 64); // get last possible error
			if(strlen(strbuf)>5) trace(strbuf);

			if (samples_sent != num_tx_samps) {
				/*UHD_LOG_ERROR*/ std::cerr <<  /*"TX-STREAM",*/
					"The tx_stream timed out sending " << num_tx_samps << " samples ("
					<< samples_sent << " sent)." << std::endl;
				return;
			}

			if ((loop_n % 1000) == 0) get_gps_str();
			loop_n++;
		}

	}
}

int UHD_SAFE_MAIN(int argc, char* argv[])
{
	// variables to be set by po
	/*std::string args, file, type, ant, subdev, ref, wirefmt, channel;
	size_t spb;
	double rate, freq, gain, bw, delay, lo_offset;*/

	// setup the program options
	//po::options_description desc("Allowed options");
	// clang-format off
	/*desc.add_options()
		("help", "help message")
		("args", po::value<std::string>(&args)->default_value(""), "multi uhd device address args")
		("file", po::value<std::string>(&file)->default_value("usrp_samples.dat"), "name of the file to read binary samples from")
		("type", po::value<std::string>(&type)->default_value("short"), "sample type: double, float, or short")
		("spb", po::value<size_t>(&spb)->default_value(10000), "samples per buffer")
		("rate", po::value<double>(&rate), "rate of outgoing samples")
		("freq", po::value<double>(&freq), "RF center frequency in Hz")
		("lo-offset", po::value<double>(&lo_offset)->default_value(0.0),
			"Offset for frontend LO in Hz (optional)")
		("gain", po::value<double>(&gain), "gain for the RF chain")
		("ant", po::value<std::string>(&ant), "antenna selection")
		("subdev", po::value<std::string>(&subdev), "subdevice specification")
		("bw", po::value<double>(&bw), "analog frontend filter bandwidth in Hz")
		("ref", po::value<std::string>(&ref)->default_value("internal"), "reference source (internal, external, mimo)")
		("wirefmt", po::value<std::string>(&wirefmt)->default_value("sc16"), "wire format (sc8 or sc16)")
		("delay", po::value<double>(&delay)->default_value(0.0), "specify a delay between repeated transmission of file (in seconds)")
		("channel", po::value<std::string>(&channel)->default_value("0"), "which channel to use")
		("repeat", "repeatedly transmit file")
		("int-n", "tune USRP with integer-n tuning")
	;
	// clang-format on
	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm);

	// print the help message
	if (vm.count("help")) {
		std::cout << boost::format("UHD TX samples from file %s") % desc << std::endl;
		return ~0;
	}
*/

	std::map</*std::string */ char *, char *> vm;

	vm["args"] = "";
	vm["file"] = "";
	vm["type"] = "short"; 
	vm["spb"] = "10000"; 
	vm["rate"] = "0";
	vm["freq"] = "0";
	vm["lo-offset"] = "0.0"; 
	vm["gain"] = "0";
	vm["ant"] = "";
	vm["subdev"] = ""; 
	vm["bw"] = "0"; 
	vm["ref"] = "internal";
	vm["wirefmt"] = "sc16";
	vm["delay"] = "0.0";
	vm["channel"] = "0";
	vm["repeat"] = "0";
	vm["int-n"] = "0";

	// add, error criteria
	vm["eavg"] = "20";
	vm["econd"] = "30000";

	for (int i = 1; i < argc; i++) {

		std::cout << boost::format("input: %s") % argv[i] << std::endl;

		if (!strcmp(argv[i], "--args")) { vm["args"] = argv[++i]; continue; }
		if (!strcmp(argv[i], "--file")) { vm["file"] = argv[++i]; continue; }
		if (!strcmp(argv[i], "--type")) { vm["type"] = argv[++i]; continue; }
		if (!strcmp(argv[i], "--spb")) { vm["spb"] = argv[++i]; continue; }
		if (!strcmp(argv[i], "--rate")) { vm["rate"] = argv[++i]; continue; }
		if (!strcmp(argv[i], "--freq")) { vm["freq"] = argv[++i]; continue; }
		if (!strcmp(argv[i], "--lo-offset")) { vm["lo-offset"] = argv[++i]; continue; }
		if (!strcmp(argv[i], "--gain")) { vm["gain"] = argv[++i]; continue; }
		if (!strcmp(argv[i], "--ant")) { vm["ant"] = argv[++i]; continue; }
		if (!strcmp(argv[i], "--subdev")) { vm["subdev"] = argv[++i]; continue; }
		if (!strcmp(argv[i], "--bw")) { vm["bw"] = argv[++i]; continue; }
		if (!strcmp(argv[i], "--ref")) { vm["ref"] = argv[++i]; continue; }
		if (!strcmp(argv[i], "--wirefmt")) { vm["wirefmt"] = argv[++i]; continue; }
		if (!strcmp(argv[i], "--delay")) { vm["delay"] = argv[++i]; continue; }
		if (!strcmp(argv[i], "--channel")) { vm["channel"] = argv[++i]; continue; }
		if (!strcmp(argv[i], "--repeat")) vm["repeat"] = "1";
		if (!strcmp(argv[i], "--int-n")) vm["int-n"] = "1";

		if (!strcmp(argv[i], "--eavg")){ vm["eavg"] = argv[++i]; continue; }
		if (!strcmp(argv[i], "--econd")){ vm["econd"] = argv[++i]; continue; }
	}

	std::string args(vm["args"]), file(vm["file"]), type(vm["type"]), ant(vm["ant"]), subdev(vm["subdev"]), ref(vm["ref"]), wirefmt(vm["wirefmt"]), channel(vm["channel"]);
	size_t spb = atoi(vm["spb"]);
	double rate = atof(vm["rate"]), freq = atof(vm["freq"]), gain = atof(vm["gain"]), bw = atof(vm["bw"]), delay = atof(vm["delay"]), lo_offset = atof(vm["lo-offset"]);

	//std::cout << boost::format("rate: %s %f") % ((const char*)vm["rate"]) % rate << std::endl;


	bool repeat = atoi(vm["repeat"]) > 0;

	eavg = atoi(vm["eavg"]);              std::cout << boost::format("eavg=%i") % eavg << std::endl;
	econd = atoi(vm["econd"]);            std::cout << boost::format("econd=%i") % econd << std::endl;


	// redirect STDOUT to file:
	freopen("tx_samples.log", "a", stdout);
	freopen("tx_samples_err.log", "a", stderr);


	// create a usrp device
	std::cout << std::endl;
	std::cout << boost::format("Creating the usrp device with: %s...") % args << std::endl;

	trace("[start] uhd::usrp::multi_usrp::make");
	usrp = uhd::usrp::multi_usrp::make(args);
	trace("[end] uhd::usrp::multi_usrp::make");



	//std::cout << boost::format("\nCreating the clock device with: %s...\n") % args;
	//usrpclock = uhd::usrp_clock::multi_usrp_clock::make(args);

	// Verify GPS sensors are present
	/*std::vector<std::string> clock_sensor_names = usrpclock->get_sensor_names(0);
	if (std::find(clock_sensor_names.begin(), clock_sensor_names.end(), "gps_locked") == clock_sensor_names.end()) {
		std::cout << "\ngps_locked sensor not found.  This could mean that this unit does not have a GPSDO.\n\n";
		return EXIT_FAILURE;
	}*/





	try {
		trace("[start] usrp->get_num_mboards");
		size_t num_mboards = usrp->get_num_mboards();
		trace("[end] usrp->get_num_mboards");

		if (ref != "internal") {

			size_t num_gps_locked = 0;
			for (size_t mboard = 0; mboard < num_mboards; mboard++) {
				std::cout << "Synchronizing mboard " << mboard << ": "
					<< usrp->get_mboard_name(mboard) << std::endl;

				// Set references to GPSDO
				trace("[start] usrp->set_clock_source");
				usrp->set_clock_source("gpsdo", mboard);
				usrp->set_time_source("gpsdo", mboard);
				trace("[end] usrp->set_clock_source");

				std::cout << std::endl;

				// Check for 10 MHz lock
				trace("[start] usrp->get_mboard_sensor_names");
				std::vector<std::string> sensor_names = usrp->get_mboard_sensor_names(mboard);
				trace("[end] usrp->get_mboard_sensor_names");

				if (std::find(sensor_names.begin(), sensor_names.end(), "ref_locked")
					!= sensor_names.end()) {
					std::cout << "Waiting for reference lock..." << std::flush;
					bool ref_locked = false;
					for (int i = 0; i < 30 and not ref_locked; i++) {

						trace("[start] usrp->get_mboard_sensor(ref_locked)");
						ref_locked = usrp->get_mboard_sensor("ref_locked", mboard).to_bool();
						trace("[end] usrp->get_mboard_sensor");

						if (not ref_locked) {
							std::cout << "." << std::flush;
							std::this_thread::sleep_for(std::chrono::seconds(1));
						}
					}
					if (ref_locked) {
						std::cout << "LOCKED" << std::endl;
					}
					else {
						std::cout << "FAILED" << std::endl;
						std::cout << "Failed to lock to GPSDO 10 MHz Reference. Exiting."
							<< std::endl;
						exit(EXIT_FAILURE);
					}
				}
				else {
					std::cout << boost::format(
						"ref_locked sensor not present on this board.\n");
				}

				// Wait for GPS lock
				trace("[start]  usrp->get_mboard_sensor(gps_locked)");
				bool gps_locked = usrp->get_mboard_sensor("gps_locked", mboard).to_bool();
				trace("[end]  usrp->get_mboard_sensor");
				if (gps_locked) {
					num_gps_locked++;
					std::cout << boost::format("GPS Locked\n");
				}
				else {
					std::cerr
						<< "WARNING:  GPS not locked - time will not be accurate until locked"
						<< std::endl;
				}

				// Set to GPS time
				trace("[start] usrp->get_mboard_sensor(gps_time)");
				uhd::time_spec_t gps_time = uhd::time_spec_t(
					int64_t(usrp->get_mboard_sensor("gps_time", mboard).to_int()));
				trace("[end] usrp->get_mboard_sensor");

				trace("[start] usrp->set_time_next_pps");
				usrp->set_time_next_pps(gps_time + 1.0, mboard);
				trace("[end] usrp->set_time_next_pps");

				// Wait for it to apply
				// The wait is 2 seconds because N-Series has a known issue where
				// the time at the last PPS does not properly update at the PPS edge
				// when the time is actually set.
				std::this_thread::sleep_for(std::chrono::seconds(2));

				// Check times
				trace("[start] usrp->get_mboard_sensor(gps_time)");
				gps_time = uhd::time_spec_t(
					int64_t(usrp->get_mboard_sensor("gps_time", mboard).to_int()));
				trace("[end] usrp->get_mboard_sensor");

				uhd::time_spec_t time_last_pps = usrp->get_time_last_pps(mboard);
				std::cout << "USRP time: "
					<< (boost::format("%0.9f") % time_last_pps.get_real_secs())
					<< std::endl;
				std::cout << "GPSDO time: "
					<< (boost::format("%0.9f") % gps_time.get_real_secs()) << std::endl;
				if (gps_time.get_real_secs() == time_last_pps.get_real_secs())
					std::cout << std::endl
					<< "SUCCESS: USRP time synchronized to GPS time" << std::endl
					<< std::endl;
				else
					std::cerr << std::endl
					<< "ERROR: Failed to synchronize USRP time to GPS time"
					<< std::endl
					<< std::endl;
			} // for mboards

			if (num_gps_locked == num_mboards and num_mboards > 1) {
				// Check to see if all USRP times are aligned
				// First, wait for PPS.
				uhd::time_spec_t time_last_pps = usrp->get_time_last_pps();
				while (time_last_pps == usrp->get_time_last_pps()) {
					std::this_thread::sleep_for(std::chrono::milliseconds(1));
				}

				// Sleep a little to make sure all devices have seen a PPS edge
				std::this_thread::sleep_for(std::chrono::milliseconds(200));

				// Compare times across all mboards
				bool all_matched = true;
				uhd::time_spec_t mboard0_time = usrp->get_time_last_pps(0);
				for (size_t mboard = 1; mboard < num_mboards; mboard++) {
					uhd::time_spec_t mboard_time = usrp->get_time_last_pps(mboard);
					if (mboard_time != mboard0_time) {
						all_matched = false;
						std::cerr << (boost::format("ERROR: Times are not aligned: USRP "
							"0=%0.9f, USRP %d=%0.9f")
							% mboard0_time.get_real_secs() % mboard
							% mboard_time.get_real_secs())
							<< std::endl;
					}
				}
				if (all_matched) {
					std::cout << "SUCCESS: USRP times aligned" << std::endl << std::endl;
				}
				else {
					std::cout << "ERROR: USRP times are not aligned" << std::endl
						<< std::endl;
				}
			} // if all mbords time ok
		} // if not internal
	}
	catch (std::exception& e) {
		std::cout << boost::format("\nError: %s") % e.what();
		std::cout << boost::format(
			"This could mean that you have not installed the GPSDO correctly.\n\n");
		std::cout << boost::format("Visit one of these pages if the problem persists:\n");
		std::cout << boost::format(
			" * N2X0/E1X0: http://files.ettus.com/manual/page_gpsdo.html");
		std::cout << boost::format(
			" * X3X0: http://files.ettus.com/manual/page_gpsdo_x3x0.html\n\n");
		std::cout << boost::format(
			" * E3X0: http://files.ettus.com/manual/page_usrp_e3x0.html#e3x0_hw_gps\n\n");
		exit(EXIT_FAILURE);
	}







	// Lock mboard clocks
	if (vm.count("ref")) {
		std::cout << boost::format("set ref to: %s") % ((const char*)vm["ref"]) << std::endl;
		trace("[start] usrp->set_clock_source(ref)");
		usrp->set_clock_source(ref);
		trace("[end] usrp->set_clock_source(ref)");
	}

	// always select the subdevice first, the channel mapping affects the other settings
	if (vm.count("subdev"))
		usrp->set_tx_subdev_spec(subdev);

	std::cout << boost::format("Using Device: %s") % usrp->get_pp_string() << std::endl;

	// set the sample rate
	if (/*not vm.count("rate") || !atoi(vm["rate"])*/ rate==0.0) {
        std::cerr << "Please specify the sample rate with --rate" << std::endl;
        return ~0;
    }
    std::cout << boost::format("Setting TX Rate: %f Msps...") % (rate / 1e6) << std::endl;
    usrp->set_tx_rate(rate);
    std::cout << boost::format("Actual TX Rate: %f Msps...") % (usrp->get_tx_rate() / 1e6)
              << std::endl
              << std::endl;

    // set the center frequency
    if (/*not vm.count("freq") || !atoi(vm["freq"])*/ freq==0.0) {
        std::cerr << "Please specify the center frequency with --freq" << std::endl;
        return ~0;
    }
    std::cout << boost::format("Setting TX Freq: %f MHz...") % (freq / 1e6) << std::endl;
    std::cout << boost::format("Setting TX LO Offset: %f MHz...") % (lo_offset / 1e6)
              << std::endl;
    uhd::tune_request_t tune_request;
    tune_request = uhd::tune_request_t(freq, lo_offset);
    if (atoi(vm["int-n"]))
        tune_request.args = uhd::device_addr_t("mode_n=integer");
    usrp->set_tx_freq(tune_request);
    std::cout << boost::format("Actual TX Freq: %f MHz...") % (usrp->get_tx_freq() / 1e6)
              << std::endl
              << std::endl;

    // set the rf gain
    if (vm.count("gain")) {
        std::cout << boost::format("Setting TX Gain: %f dB...") % gain << std::endl;
        usrp->set_tx_gain(gain);
        std::cout << boost::format("Actual TX Gain: %f dB...") % usrp->get_tx_gain()
                  << std::endl
                  << std::endl;
    }

    // set the analog frontend filter bandwidth
    if (vm.count("bw")) {
        std::cout << boost::format("Setting TX Bandwidth: %f MHz...") % (bw / 1e6)
                  << std::endl;
        usrp->set_tx_bandwidth(bw);
        std::cout << boost::format("Actual TX Bandwidth: %f MHz...")
                         % (usrp->get_tx_bandwidth() / 1e6)
                  << std::endl
                  << std::endl;
    }

    // set the antenna
    if (vm.count("ant"))
        usrp->set_tx_antenna(ant);

    // allow for some setup time:
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Check Ref and LO Lock detect
    std::vector<std::string> sensor_names;
    sensor_names = usrp->get_tx_sensor_names(0);
    if (std::find(sensor_names.begin(), sensor_names.end(), "lo_locked")
        != sensor_names.end()) {

		trace("[start] usrp->get_tx_sensor(lo_locked)");
        uhd::sensor_value_t lo_locked = usrp->get_tx_sensor("lo_locked", 0);
		trace("[end] usrp->get_tx_sensor");

        std::cout << boost::format("Checking TX: %s ...") % lo_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(lo_locked.to_bool());
    }
    sensor_names = usrp->get_mboard_sensor_names(0);
    if ((ref == "mimo")
        and (std::find(sensor_names.begin(), sensor_names.end(), "mimo_locked")
             != sensor_names.end())) {

		trace("[start] usrp->get_mboard_sensor(mimo_locked)");
        uhd::sensor_value_t mimo_locked = usrp->get_mboard_sensor("mimo_locked", 0);
		trace("[end] usrp->get_mboard_sensor");

        std::cout << boost::format("Checking TX: %s ...") % mimo_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(mimo_locked.to_bool());
    }
    if ((ref == "external")
        and (std::find(sensor_names.begin(), sensor_names.end(), "ref_locked")
             != sensor_names.end())) {

		trace("[start] usrp->get_mboard_sensor(ref_locked)");
        uhd::sensor_value_t ref_locked = usrp->get_mboard_sensor("ref_locked", 0);
		trace("[end] usrp->get_mboard_sensor");

        std::cout << boost::format("Checking TX: %s ...") % ref_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(ref_locked.to_bool());
    }

    // set sigint if user wants to receive
    if (repeat) {
        std::signal(SIGINT, &sig_int_handler);
        std::cout << "Press Ctrl + C to stop streaming..." << std::endl;
    }

    // create a transmit streamer
    std::string cpu_format;
    std::vector<size_t> channel_nums;
    if (type == "double")
        cpu_format = "fc64";
    else if (type == "float")
        cpu_format = "fc32";
    else if (type == "short")
        cpu_format = "sc16";
    uhd::stream_args_t stream_args(cpu_format, wirefmt);
    channel_nums.push_back(boost::lexical_cast<size_t>(channel));
    stream_args.channels             = channel_nums;
    uhd::tx_streamer::sptr tx_stream = usrp->get_tx_stream(stream_args);

    // send from file
    do {
        if (type == "double")
            send_from_file<std::complex<double>>(tx_stream, file, spb);
        else if (type == "float")
            send_from_file<std::complex<float>>(tx_stream, file, spb);
        else if (type == "short")
            send_from_file<std::complex<short>>(tx_stream, file, spb);
        else
            throw std::runtime_error("Unknown type " + type);

        if (repeat and delay > 0.0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(int64_t(delay * 1000)));
        }
    } while (repeat and not stop_signal_called);

    // finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;

	

    return EXIT_SUCCESS;
}