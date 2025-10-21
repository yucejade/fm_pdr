// SPDX-License-Identifier: GPL-2.0-or-later
// SPDX-FileCopyrightText: 2023 Kent Gibson <warthog618@gmail.com>

/* Minimal example of toggling a single line. */

#include <cstdlib>
#include <chrono>
#include <filesystem>
#include <gpiod.hpp>
#include <iostream>
#include <thread>

namespace {

/* Example configuration - customize to suit your situation. */
const ::std::filesystem::path chip_path("/dev/gpiochip0");
const ::gpiod::line::offset line_offset = 5;

::gpiod::line::value toggle_value(::gpiod::line::value v)
{
	return (v == ::gpiod::line::value::ACTIVE) ?
		       ::gpiod::line::value::INACTIVE :
		       ::gpiod::line::value::ACTIVE;
}

} /* namespace */

int main()
{
	::gpiod::line::value value = ::gpiod::line::value::ACTIVE;

	auto request =
		::gpiod::chip(chip_path)
			.prepare_request()
			.set_consumer("toggle-line-value")
			.add_line_settings(
				line_offset,
				::gpiod::line_settings().set_direction(
					::gpiod::line::direction::OUTPUT))
			.do_request();

	for (;;) {
		::std::cout << line_offset << "=" << value << ::std::endl;

		std::this_thread::sleep_for(std::chrono::seconds(1));
		value = toggle_value(value);
		request.set_value(line_offset, value);
	}
}
