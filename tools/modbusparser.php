#!/usr/bin/php 
<?php
/*
        reads p300 debug(1) output from stdin
*/

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
                                                // sende und empfangsdaten vorhanden
                                                if (($data[0][1]==3) && ($data[1][1]==3))
                                                {
                                                        echo "$name read ";
                                                        $start = convertword($data[0], 2);
                                                        $count = convertword($data[0],4);
                                                        for ($i=0;$i<$count;$i++)
                                                        {
                                                                echo (int)($i+$start)."=".convertword($data[1],3+($i*2))." ";
                                                        }
                                                        echo "\n";
                                                }
                                                if (($data[0][1]==16) && ($data[1][1]==16))
                                                {
                                                        echo "$name write ";
                                                        $start = convertword($data[0], 2);
                                                        $count = convertword($data[0],4);
                                                        for ($i=0;$i<$count;$i++)
                                                        {
                                                                echo (int)($i+$start)."=".convertword($data[0],7+($i*2))." ";
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
}
fclose($stdin);
//print_r($comm);

function convertword($data, $address)
{
        if ( (isset($data[$address])) && (isset($data[$address+1]))) return ($data[$address]*256)+$data[$address+1];
                else return -1;
}
