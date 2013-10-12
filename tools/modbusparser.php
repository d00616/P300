#!/usr/bin/php 
<?php

$txt=array();
function restisterparser($address, $value, &$txt = false )
{
 if (!$txt) $txt=array();
 
 if (($address>=0x400) && ($address!=0x413))
 {
         restisterparser( ($address-0x400)*2, $value >> 8, &$txt );
         restisterparser( ($address-0x400)*2+1, $value & 0xff, &$txt );
 }
 
 switch ($address)
 {
         case 0:
                 $txt[]="Stufe:".$value;
                 break;
         case 1:
                 $txt[]="Volumenstrom:".$value;
                 break;
         case 2:
                 $wd=array(
                         1 => 'Mo',
                         2 => 'Tu',
                         4 => 'We',
                         8 => 'Th',
                        16 => 'Fr',
                        32 => 'Sa',
                        64 => 'Su'
                         
                 );
                 $txt[]="Weekday:".$wd[$value];
                 break;
         case 3:
                 $txt[]="Hour:".$value;
                 break;
         case 4:
                 $txt[]="Minute:".$value;
                 break;
         case 7:
                 if ($value==1) $txt[]="Kalibriert"; else $txt[]="Unkalibriert (".$value.")";
                 break;
         case 13:
                 if ($value==4) $txt[]="Raumluftabh. Feuerstaette"; else 
                 if ($value==0) $txt[]="Geschlossene Feuerstaette"; else
                                $txt[]="?";
                 break;
         case 14:
                 $txt[]="Bypass Max:".$value;
                 break;
         case 15:
                 $txt[]="Bypass Min:".$value;
                 break;
         case 16:
         case 17:
         case 18:
         case 19:
                 $txt[]="T".($address-15).":".$value;
                 break;
         case 20:
         case 21:
                 $txt[]="S".($address-19).":".$value*2;
                 break;
         case 24:
                 $txt[]="Heizregister:".$value;
                 break;
         case 0x413:
                 $txt[]="Countdown:".$value;
                 break;
        default:
                if ($address<1024)
                {
                        $txt[]=$address."=".$value;
                }
 }

 if (count($txt)>0) return "(".implode(",",$txt).")";
}

$comm=array();

$stdin = fopen('php://stdin', 'r');
while ($line = preg_replace("/ +/"," ",fgets($stdin)))
{
        if (substr($line,0,2)=="D ")
        {
                $tmp = explode(" ",$line);
                //print "$tmp[1] $tmp[2] $tmp[3] $tmp[4]\n";
                if (count($tmp)>3)
                {
                        if (($tmp[2]==">") && ($tmp[3]=='P300')) $comm[$tmp[1]][0][]=hexdec($tmp[4]);
                        if (($tmp[2]=="<") && ($tmp[3]=='BUFFER')) $comm[$tmp[1]][1][]=hexdec($tmp[4]);
                }
                
                // Ende eines packetes
                if (count($tmp)>2)
                {
                        if ( ($tmp[1]=="proxy_source=NULL") || ($tmp[1]=="Reset"))
                        {
                                foreach ( $comm as $name=>$data)
                                {
                                        if ( (isset($data[0])) && (isset($data[1])))
                                        {
                                                // Startregister
                                                $start = convertword($data[0],2);
                                                // Anzahl der Register
                                                $count = convertword($data[0],4);                                                
  
                                                // sende und empfangsdaten vorhanden
                                                if (($data[0][1]==3) && ($data[1][1]==3))
                                                {
                                                        echo "$name read ";
                                                        for ($i=0;$i<$count;$i++)
                                                        {
                                                                $val=convertword($data[1],3+($i*2));
                                                                echo sprintf("%x=%x%s ",(int)($i+$start),$val,restisterparser($i+$start,$val));
                                                        }
                                                        echo "\n";
                                                }
                                                if (($data[0][1]==16) && ($data[1][1]==16))
                                                {
                                                        echo "$name write ";
                                                        for ($i=0;$i<$count;$i++)
                                                        {
                                                                $val= convertword($data[0],7+($i*2));
                                                                echo sprintf("%x=%x%s ",(int)($i+$start),$val,restisterparser($i+$start,$val));
                                                        }
                                                        echo "\n";
                                                }
                                                
                                                // reset
                                                $comm[$name]=array();
                                        }
                                }
                        }
                } 
                
        }
                else echo $line;
}
fclose($stdin);
//print_r($comm);

function convertword($data, $address)
{
        if ( (isset($data[$address])) && (isset($data[$address+1]))) return (($data[$address]*256)+$data[$address+1]);
                else return -1;
}
