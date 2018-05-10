Command Example

1.SETTING COMMAND

 send Broadcast packet to change sending rate to 44(command id=12)
> SW 02 FFFF 12 01 01 00 44 EW

 send unicast packet to A69D change sending rate to 44(command id=12)
> SW 02 A69D 12 01 01 00 44 EW

 send unicast packet to A69D change sending rate to 44(command id=12) and change temperature threshold to 40
> SW 02 A69D 12 02 01 00 44 02 01 40 EW

2.ASK COMMAND

 send Broadcast packet to ask mote's configuration (command id =12)
> SW 01 FFFF 12 EW

 send unicast packet to ask a69d's configuration (command id =12)
> SW 01 A69D 12 EW

