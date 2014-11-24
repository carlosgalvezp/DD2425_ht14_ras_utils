#include <ras_utils/ras_utils.h>

namespace RAS_Utils
{

    int sign(double a)
    {
        if (a >= 0) return 1;
        else        return -1;
    }

    double time_diff_ms(const std::clock_t &begin, const std::clock_t &end)
    {
        return 1000.0*(double)(end-begin)/(double)CLOCKS_PER_SEC;
    }

    double time_diff_ms(timeval *begin, timeval *end)
    {
        return 1000.0*(end->tv_sec  - begin->tv_sec) +
                0.001*(end->tv_usec - begin->tv_usec);
    }

    double time_diff_ms(const ros::WallTime &begin, const ros::WallTime &end)
    {
        return (end.toNSec() - begin.toNSec())/1000000.0;
    }

    double time_diff_ns(const ros::WallTime &begin, const ros::WallTime &end)
    {
        return (end.toNSec() - begin.toNSec());
    }


    double sensorToDistanceInCM(int sensor_val, std::vector<double> polynomial_coof)
    {
        double distance = 0;
        for(int n = 0; n < polynomial_coof.size(); n++) {
            distance += polynomial_coof[n] * pow(sensor_val, n);
        }
        return distance;
    }

    double shortSensorToDistanceInCM(int sensor_val)
    {
        if(sensor_val > 440) {
            return 4.5;
        }


        return sensorToDistanceInCM(sensor_val, {
                                    262.503338214074,
                                    -4.88675316092840,
                                    0.0415096360540543,
                                    -0.000191007269073580,
                                    4.90926172893023e-07,
                                    -6.62146190833533e-10,
                                    3.64763123318473e-13
                                    });
    }

    double longSensorToDistanceInCM(int sensor_val)
    {
        return sensorToDistanceInCM(sensor_val, {
                                    329.045444431424,
                                    -5.68757150017330,
                                    0.0447961028587077,
                                    -0.000188802004989696,
                                    4.37076393642977e-07,
                                    -5.23382644623806e-10,
                                    2.53149410424676e-13,
                                    });
    }

    void print(const std::string & text) {
         std::cout << text.c_str() << std::endl;
    }

    void print(const std::string & text, const double value, std::string & padding) {
        std::cout << text.c_str() << ":" << padding.c_str() << value << std::endl;
    }

    void print(const std::string & text, const double value) {
        std::string padding;
        padding.assign(PRINT_PADDING, ' ');
        std::cout << text.c_str() << ":" << padding.c_str() << value << std::endl;
    }
    void print(const std::string & text, const double value1, const double value2) {
        std::string padding;
        padding.assign(PRINT_PADDING, ' ');
        std::cout << text.c_str() << ":" << padding.c_str() << value1 << "|" << value2 << std::endl;
    }

    void print(const std::vector<std::string> & texts, const std::vector<double> & values) {
        if(texts.size() != values.size()) {
            print("!!! ERROR !!! Vectors in print function have different sizes");
            print("texts, values", texts.size(), values.size());
            return;
        }
        int longest_text_length = 0;
        for(std::string text : texts) {
            if(text.length() > longest_text_length) {
                longest_text_length = text.length();
            }
        }

        std::string padding;


        for(int i = 0; i < texts.size(); i++) {
            int missing = longest_text_length + PRINT_PADDING - texts[i].length();
            padding.assign(missing, ' ');
            print(texts[i], values[i], padding);
        }
        std::cout << std::endl;
    }

    double mean(const std::vector<double> &data)
    {
        double res=0;
        double N = data.size();
        for(std::size_t i = 0; i < N; ++i)
        {
            res += data[i];
        }
        return res/N;
    }

    double std(const std::vector<double> &data, double mu)
    {
        double res=0;
        double N = data.size();
        for(std::size_t i = 0; i < N; ++i)
        {
            res += (data[i] - mu)*(data[i] - mu);
        }
        return sqrt(res/N);
    }

    double std(const std::vector<double> &data)
    {
        return std(data, mean(data));
    }

    double mahalanobis_distance(const double &x, const double &mu, const double &sigma)
    {
        return fabs(x-mu)/sigma;
    }

}
