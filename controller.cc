#include <stdio.h>
#include <algorithm>

#include "controller.hh"
#include "timestamp.hh"

using namespace Network;

const double alpha = 2;
const double beta = 2;
const double rtt_weight = 0.7;
const double throughput_weight = 0.7;
const uint64_t tick_time = 100; // 100 ms between updates

double aimd_wind; //in packets
double rtt; //in ms
double curr_throughput;

uint64_t last_seq_num;
uint64_t last_packet_time;

/* Default constructor */
Controller::Controller( const bool debug )
  : debug_( debug )
{
  rtt = 100; //initial rtt estimate
  last_seq_num = 0;
  last_packet_time = 0;
  curr_throughput = 0.4; // some reasonable starting estimate
}

/* Get current window size, in packets */
unsigned int Controller::window_size( void )
{
  int the_window_size = (int)(tick_time * curr_throughput);

  if ( debug_ ) {
    fprintf( stderr, "At time %lu, rtt %f, throughput %f, return window_size = %d.\n",
	     timestamp(), rtt, curr_throughput, the_window_size );
  }

  return the_window_size;
}

/* A packet was sent */
void Controller::packet_was_sent( const uint64_t sequence_number,
				  /* of the sent packet */
				  const uint64_t send_timestamp )
                                  /* in milliseconds */
{
  /* Default: take no action */
  if ( debug_ ) {
    fprintf( stderr, "At time %lu, sent packet %lu.\n",
	     send_timestamp, sequence_number );
  }
}

/* An ack was received */
void Controller::ack_received( const uint64_t sequence_number_acked,
			       /* what sequence number was acknowledged */
			       const uint64_t send_timestamp_acked,
			       /* when the acknowledged packet was sent */
			       const uint64_t recv_timestamp_acked,
			       /* when the acknowledged packet was received */
			       const uint64_t timestamp_ack_received )
                               /* when the ack was received (by sender) */
{
  uint64_t new_rtt = timestamp_ack_received - send_timestamp_acked;
  rtt = (rtt_weight * rtt) + (1-rtt_weight)*(new_rtt);

  uint64_t wait_time = timestamp_ack_received - last_packet_time;
  if (wait_time >= tick_time) {
    uint64_t packets_passed = sequence_number_acked - last_seq_num;
    double ewma_throughput = (throughput_weight*curr_throughput) + 
      (1 - throughput_weight)*((float)packets_passed/wait_time);

    // in case we get less than 1 packet sent
    curr_throughput = std::max(1.0/rtt, ewma_throughput); 
    fprintf(stderr, "new rtt %lu rtt %f wait time %lu ewma throughput %f curr throughput %f packets passed %lu\n", new_rtt, rtt, wait_time, ewma_throughput, curr_throughput, packets_passed);

    last_seq_num = sequence_number_acked;
    last_packet_time = timestamp_ack_received;
  }
  
  if ( debug_ ) {
    fprintf( stderr, "At time %lu, received ACK for packet %lu",
	     timestamp_ack_received, sequence_number_acked );

    fprintf( stderr, " (sent %lu, received %lu by receiver's clock).\n",
	     send_timestamp_acked, recv_timestamp_acked );
  }
}

/* How long to wait if there are no acks before sending one more packet */
unsigned int Controller::timeout_ms( void )
{
  return 1000; /* timeout of one second */
}
