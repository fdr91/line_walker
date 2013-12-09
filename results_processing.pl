use strict;

my $input=$ARGV[0];

open (IN, "<$input") or die "can't open file";
open (OUT, ">output") or die "can't open file";

my $prev_time;

my $def=0;
my $summ=0;

my $i;

for($i=0, <IN>,$i++)
{
	$i++;	
	if($_=~m/\d+\scur_time=\d+/)
	{	
		my @arr=split('=', $&);
		if(! defined $prev_time)
		{
			$prev_time=$arr[1];
			next;
		}
		my $cur_time=@arr[1];
		$def+=$cur_time-$prev_time;
	}
	if($i>1000)
	{
		last;
	}
}

$def/=999000000000;

print "\n".$def."\n";
