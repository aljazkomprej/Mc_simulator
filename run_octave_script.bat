@echo off
:: Author: stojand

:: Define path of your octave \bin folder
set PATH=%PATH%;D:\Programs\Octave\Octave-4.2.1\bin

pushd D:\stojand\Projekti\WaPu_FOC\c programming\mcsim20170928_jc\mcsim20170928_ds\mcsim20170928\mcsim

start octave-cli --persist risi_inverter.m
