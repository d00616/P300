#!/usr/bin/php
<?
/*
 lesbaren dump erzeugen
 > function prmb { print z,"=",modbus(z); z++; };
 > z=0;
 > run prmb,200
 Bei 1023 strg+c
 > startup
 
 */
$map=array();
$stdin = fopen('php://stdin', 'r');
while ($line = fgets($stdin))
{
 $tmp = explode("=", $line);
 if (count($tmp)==2) $map[(int)$tmp[0]]=(int)$tmp[1];
}


$perline=16;
for ($i=0;$i<1024;$i=$i+$perline)
{
 printf("%03x ",$i);
 for ($j=0;$j<$perline;$j++) printf("%02x ",$map[$i+$j]);
 printf(" ");
 for ($j=0;$j<$perline;$j++) { $c=$map[$i+$j]; if ($c>30) printf("%c",$c); else printf("."); }
 printf("\n");
}
