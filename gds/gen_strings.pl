#! /usr/bin/perl -w
use strict;

# Input file:
#    IMANUFACT "Link Engineering"
#    IPRODUCT "Gas Deck Shield"
#    ISERIALNUM "GDS00001"

my %strings = (
  IMANUFACT => "Link Engineering",
  IPRODUCT => "Gas Deck Shield",
  ISERIALNUM => "GDS00005"
);
my %is_string = map { ($_, 1) } qw(IMANUFACT IPRODUCT ISERIALNUM);
my $has_a_string = 0;

sub def_strings {
  my ($name, $value) = @_;
  if ($is_string{$name} && $value) {
    $has_a_string = 1;
    print "  <Value>CONF_USB_CDCD_ACM_${name}_EN=1</Value>\n";
    my @vals = map { (ord($_),0) } split(//, $value);
    my $desclen = @vals + 2;
    print "  <Value>CONF_USB_CDCD_ACM_${name}_STR_DESC=,$desclen,3,",
          join(',',@vals), "</Value>\n";
  }
}

for my $name ( keys %strings ) {
  def_strings($name, $strings{$name});
}

if ($has_a_string) {
  print "  <Value>CONF_USB_CDCD_ACM_LANGID_DESC=4,3,9,4</Value>\n"
}