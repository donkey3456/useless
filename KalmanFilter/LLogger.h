#pragma once

#include <iostream>
#include <sstream>
#include <mutex>

class LLogger 
{
	class PrintThread : public std::ostringstream
	{
	public:
		PrintThread(std::mutex& mutex, std::ostream& stream, std::string prefix = "", bool verbose = true)
			:_mutexPrint(mutex), _output_stream(stream), _prefix(prefix), _verbose(verbose) {}

		PrintThread(const PrintThread& p)
			:_mutexPrint(p._mutexPrint), _output_stream(p._output_stream),
			_prefix(p._prefix), _verbose(p._verbose) {}

		~PrintThread()
		{
			if (str().size() != 0 && _verbose)
			{
				std::lock_guard<std::mutex> guard(_mutexPrint);
				_output_stream << _prefix <<str();
				if (*str().rbegin() != '\n')
				{
					_output_stream << '\n';
				}
				_output_stream.flush();
			}
		}

	protected:
		std::mutex& _mutexPrint;
		std::ostream& _output_stream;
		std::string _prefix;
		bool _verbose;
	};

public:
	LLogger(std::ostream& stream = std::cerr);

	~LLogger();

	PrintThread Info();
	PrintThread Err();

	void EnableInfo(bool enable) { _enable_info = enable; }
	void EnableErr(bool enable) { _enable_err = enable; }
	void Redirect(std::ostream& output_stream);

protected:
	std::mutex _mutex_stream;
	std::ostream& _output_stream;
	bool _enable_info;
	bool _enable_err;
};

class LoggableObj
{
public:
	LoggableObj() {};
	virtual ~LoggableObj() {};

	void EnableInfo(bool enable) { _logger.EnableInfo(enable); }
	void EnableErr(bool enable) { _logger.EnableErr(enable); }
	void Redirect(std::ostream& output_stream) { _logger.Redirect(output_stream); }

protected:
	LLogger _logger;
};