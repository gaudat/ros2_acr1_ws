#!/bin/sh

echo -n 'Charger connected: '
cat /sys/class/power_supply/axp288_charger/online

cat /sys/class/power_supply/axp288_fuel_gauge/status

echo -n 'Capacity remaining: '
cat /sys/class/power_supply/axp288_fuel_gauge/capacity

echo -n 'Current charge: '
cat /sys/class/power_supply/axp288_fuel_gauge/charge_now

echo -n 'Full charge: '
cat /sys/class/power_supply/axp288_fuel_gauge/charge_full


echo 'Valid input voltage: 4.1V to 7V'
