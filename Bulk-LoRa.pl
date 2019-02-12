#!/usr/bin/perl -w

###################################################################################
#          Event-based simulator for LoRa-based Bulk data transmissions           #
#                                  v.2018.12.19                                   #
#                                                                                 #
# Description: The script considers a scenario where each node transmits as soon  #
# as it has a considerable amount of data. All nodes respect the 1% radio         #
# duty-cycle restriction. The lowest possible SF is assigned to each node based   #
# on its distance to the gateway and the propagation model. The script takes as   #
# input the node positions as it is defined in the read_data sub. It outputs the  #
# data collection time and the average node energy consumption.                   #
#                                                                                 #
# Assumptions/Features:                                                           #
# -- All the transmissions are performed over the same channel                    #
# -- No acknowledgments are sent                                                  #
# -- Collisions occur when two packets overlap in SF, time, and power             #
#    (capture effect)                                                             #
# -- Non-orthogonal transmissions are taken into account                          #
# -- All the nodes have the same BW/CR settings                                   #
#                                                                                 #
# author: Dr. Dimitrios Zorbas                                                    #
# email: dimzorbas@ieee.org                                                       #
# distributed under GNUv2 General Public Licence                                  #
###################################################################################

use strict;
use POSIX;
use List::Util qw[min max];
use Time::HiRes qw( time );
use Math::Random;

die "usage: ./LoRaWAN.pl terrain_file!\n" unless (@ARGV == 1);

# node attributes
my %ncoords = (); # coordinates
my %rem_data = (); # remaining data
my %consumption = (); # consumption
my %SF = (); # Spreading Factor
my %transmissions = (); # current transmission
my %retransmisssions = (); # retransmisssions per node

# simulation and propagation parameters
my $max_retr = 0; # max number of retransmisssions per packet
my @sflist = ([7,-124,-122,-116], [8,-127,-125,-119], [9,-130,-128,-122], [10,-133,-130,-125], [11,-135,-132,-128], [12,-137,-135,-129]); # sensitivities per SF/BW
my $bw = 500; # bandwidth
my $var = 3.57; # variance
my $G = 0.5; # rand(1);
my ($dref, $Ptx, $Lpld0, $Xs, $gamma) = (40, 7, 95, $var*$G, 2.08); # attenuation model parameters
my $Ptx_w = 25 * 3.5 / 1000; # 25mA, 3.5V
my @thresholds = ([6,-16,-18,-19,-19,-20], [-24,6,-20,-22,-22,-22], [-27,-27,6,-23,-25,-25], [-30,-30,-30,6,-26,-28], [-33,-33,-33,-33,6,-29], [-36,-36,-36,-36,-36,6]); # capture effect power thresholds per SF[SF] for non-orthogonal transmissions

my @pl = (100, 100, 100, 100, 100, 100); # payload size per SF (bytes)
my $v = 2; # max variance between two successive transmissions after duty cycle (secs)
my ($gw_x, $gw_y, $gw_z) = (0, 0, 0); # to be filled in read_data()
my ($terrain, $norm_x, $norm_y) = (0, 0, 0); # terrain side
my $start_time = time; # just for statistics
my $dropped = 0; # number of dropped packets
my $total_trans = 0; # expected number of transm. packets

read_data(); # read terrain file

# find the minimum SF per node
foreach my $n (keys %ncoords){
	my $d0 = distance3d($gw_x, $ncoords{$n}[0], $gw_y, $ncoords{$n}[1], $gw_z, 0);
	$SF{$n} = min_sf($n, $d0);
	print "# $n got SF$SF{$n}\n";
	my $data = 500 + int(rand(1000)); # specify here the amount of data of each node
	$rem_data{$n} = $data;
	$retransmisssions{$n} = 0;
	$total_trans += ceil($data/$pl[$SF{$n}-7]);
}

my @examined = ();
my $collection_time = 0;

# initial transmission
foreach my $n (keys %SF){
	my $start = int(rand(airtime($SF{$n})*(scalar keys %SF))*1000000)/1000000; # rounding is applied
	my $stop = $start + airtime($SF{$n});
	print "# $n will transmit from $start to $stop\n";
	$transmissions{$n} = [$start, $stop];
	$consumption{$n} += airtime($SF{$n}) * $Ptx_w;
}

# main loop
while (scalar @examined < scalar keys %SF){
	printf "# %d transmissions available \n", scalar keys %transmissions;
	# grab the earliest transmission
	my $sel_sta = 9999999999999;
	my $sel_end = 0;
	my $sel = undef;
	foreach my $n (keys %transmissions){
		my ($sta, $end) = @{$transmissions{$n}};
		if ($sta < $sel_sta){
			$sel_sta = $sta;
			$sel_end = $end;
			$sel = $n;
		}
	}
	last if (!defined $sel);
	print "# grabbed $sel, transmission at $sel_sta -> $sel_end\n";
	$collection_time = $sel_end if ($sel_end > $collection_time);
		
	# check for collisions with other transmissions (time, SF, power)
	my $d = distance3d($gw_x, $ncoords{$sel}[0], $gw_y, $ncoords{$sel}[1], $gw_z, 0);
	my $prx = $Ptx - ($Lpld0 + 10*$gamma * log10($d/$dref) + $Xs);
	my $collided = 0;
	my @coll = ();
	foreach my $n (keys %transmissions){
		my ($sta, $end) = @{$transmissions{$n}};
		next if (($rem_data{$n} <= 0) || ($n == $sel) || ($sta > $sel_end) || ($end < $sel_sta));
		my $overlap = 0;
		# time overlap
		if ( (($sel_sta >= $sta) && ($sel_sta <= $end)) || (($sel_end <= $end) && ($sel_end >= $sta)) ){
			$overlap += 1;
		}
		# SF
		if ($SF{$n} == $SF{$sel}){
			$overlap += 2;
		}
		# power 
		my $d_ = distance3d($gw_x, $ncoords{$n}[0], $gw_y, $ncoords{$n}[1], $gw_z, 0);
		my $prx_ = $Ptx - ($Lpld0 + 10*$gamma * log10($d_/$dref) + $Xs);
		if ($overlap == 3){
			if (abs($prx - $prx_) < $thresholds[$SF{$sel}-7][$SF{$n}-7]){ # both collide
				$collided += 1;
				push (@coll, $sel) if ($collided == 1);
				push (@coll, $n);
				print "# $sel collided together with $n\n";
			}elsif (($prx - $prx_) < $thresholds[$SF{$sel}-7][$SF{$n}-7]){ # n suppressed sel
				$collided += 1;
				push (@coll, $sel) if ($collided == 1);
				print "# $sel surpressed by $n\n";
			}
		}elsif ($overlap == 1){
			if (($prx - $prx_) > $thresholds[$SF{$sel}-7][$SF{$n}-7]){
				if (($prx_ - $prx) <= $thresholds[$SF{$n}-7][$SF{$sel}-7]){
					$collided += 1;
					push (@coll, $n);
					print "# $n surpressed by $sel\n";
				}
			}else{
				if (($prx_ - $prx) > $thresholds[$SF{$n}-7][$SF{$sel}-7]){
					$collided += 1;
					push (@coll, $sel) if ($collided == 1);
					print "# $sel surpressed by $n\n";
				}else{
					push (@coll, $sel) if ($collided == 1);
					push (@coll, $n);
					print "# $sel collided together with $n\n";
				}
			}
		}
	}
	# delete collided transmissions and assign new ones
	foreach my $del (@coll){
		my ($del_sta, $del_end) = @{$transmissions{$del}};
		delete $transmissions{$del};
		if ($retransmisssions{$del} < $max_retr){
			$retransmisssions{$del} += 1;
		}else{
			$dropped += 1;
			$rem_data{$del} -= $pl[$SF{$del}-7];
			$retransmisssions{$del} = 0;
			print "# $del 's packet lost!\n";
		}
		if ($rem_data{$del} > 0){
			my $payl = $pl[$SF{$del}-7];
			$payl = $rem_data{$del} if ($rem_data{$del} < $payl);
			my $at = airtime($SF{$del}, $payl);
			$del_sta += 100*$at + int(rand($v)*1000000)/1000000;
			$transmissions{$del} = [$del_sta, $del_sta+$at];
			$consumption{$del} += $at * $Ptx_w;
		}
	}
	if ($collided == 0){
		# remove this transmission
		delete $transmissions{$sel};
		$retransmisssions{$sel} = 0;
		# reduce data and assign a new transmission
		$rem_data{$sel} -= $pl[$SF{$sel}-7];
		if ($rem_data{$sel} <= 0){
			push (@examined, $sel);
		}else{
			my $payl = $pl[$SF{$sel}-7];
			$payl = $rem_data{$sel} if ($rem_data{$sel} < $payl);
			my $at = airtime($SF{$sel}, $payl);
			$sel_sta += 100*$at + int(rand($v)*1000000)/1000000;
			$transmissions{$sel} = [$sel_sta, $sel_sta+$at];
			$consumption{$sel} += $at * $Ptx_w;
		}
		print "# $sel transmitted successfully!\n";
	}
}
print "---------------------\n";


my $avg_cons = 0;
foreach my $n (keys %SF){
	$avg_cons += $consumption{$n};
}
my $finish_time = time;
print "Data collection time = $collection_time sec\n";
printf "Avg node consumption = %.5f J\n", $avg_cons/(scalar keys %SF);
printf "Packet Delivery Ratio = %.5f\n", ($total_trans - $dropped)/$total_trans;
printf "Script execution time = %.4f secs\n", $finish_time - $start_time;

sub min_sf{
	my ($n, $d0) = @_;
	my $sf = undef;
	my $bwi = bwconv($bw);
	for (my $f=7; $f<=12; $f+=1){
		my $S = $sflist[$f-7][$bwi];
		my $d = $dref * 10**( ($Ptx - $S - $Lpld0 - $Xs)/(10*$gamma) );
		if ($d > $d0){
			$sf = $f;
			$f = 13;
		}
	}
	if (!defined $sf){
		print "node $n is unreachable!\n";
		exit;
	}
	return $sf;
}

sub airtime{
	my $sf = shift;
	my $cr = 1;
	my $H = 0;       # implicit header disabled (H=0) or not (H=1)
	my $DE = 0;      # low data rate optimization enabled (=1) or not (=0)
	my $Npream = 8;  # number of preamble symbol (12.25  from Utz paper)
	my $payload = shift;
	$payload = $pl[$sf-7] if (!defined $payload);
	
	if (($bw == 125) && (($sf == 11) || ($sf == 12))){
		# low data rate optimization mandated for BW125 with SF11 and SF12
		$DE = 1;
	}
	
	if ($sf == 6){
		# can only have implicit header with SF6
		$H = 1;
	}
	
	my $Tsym = (2**$sf)/$bw;
	my $Tpream = ($Npream + 4.25)*$Tsym;
	my $payloadSymbNB = 8 + max( ceil((8.0*$payload-4.0*$sf+28+16-20*$H)/(4.0*($sf-2*$DE)))*($cr+4), 0 );
	my $Tpayload = $payloadSymbNB * $Tsym;
	return ($Tpream + $Tpayload)/1000;
}

sub bwconv{
	my $bwi = 0;
	if ($bw == 125){
		$bwi = 1;
	}elsif ($bw == 250){
		$bwi = 2;
	}elsif ($bw == 500){
		$bwi = 3;
	}
	return $bwi;
}

sub read_data{
	my $terrain_file = $ARGV[0];
	open(FH, "<$terrain_file") or die "Error: could not open terrain file $terrain_file\n";
	my @nodes = ();
	while(<FH>){
		chomp;
		if (/^# stats: (.*)/){
			my $stats_line = $1;
			if ($stats_line =~ /terrain=([0-9]+\.[0-9]+)m\^2/){
				$terrain = $1;
			}
			$norm_x = sqrt($terrain);
			$norm_y = sqrt($terrain);
		} elsif (/^# node coords: (.*)/){
			my $point_coord = $1;
			my @coords = split(/\] /, $point_coord);
			@nodes = map { /([0-9]+) \[([0-9]+\.[0-9]+) ([0-9]+\.[0-9]+)/; [$1, $2, $3]; } @coords;
		}
	}
	close(FH);
	
	($gw_x, $gw_y, $gw_z) = (sqrt($terrain)/2, sqrt($terrain)/2, 10);
	foreach my $node (@nodes){
		my ($n, $x, $y) = @$node;
		$ncoords{$n} = [$x, $y];
	}
}

sub distance3d {
	my ($x1, $x2, $y1, $y2, $z1, $z2) = @_;
	return sqrt( (($x1-$x2)*($x1-$x2))+(($y1-$y2)*($y1-$y2))+(($z1-$z2)*($z1-$z2)) );
}
