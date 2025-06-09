#pragma once

#include <glog/logging.h>

// Import common Google logging severities into global namespace
// Note: DEBUG is not a standard glog severity, using INFO for debug messages
using google::ERROR;
using google::FATAL;
using google::INFO;
using google::WARNING;

// Define DEBUG as INFO for compatibility
#define DEBUG INFO
