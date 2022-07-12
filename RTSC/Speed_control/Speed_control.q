//This file was generated from (Commercial) UPPAAL 4.0.15 rev. CB6BB307F6F681CB, November 2019

/*

*/
//NO_QUERY

/*

*/
E<> (V2.gear_reverse && SC.increase_speed)

/*

*/
E<> (V4.idle && SC.decrease_speed)

/*

*/
E<> (V3.gear3 && SC.maintain_speed)

/*

*/
E<> (V1.gear2 && SC.decrease_speed)

/*

*/
E<> (V1.gear1 && SC.increase_speed)

/*

*/
A[] not deadlock
