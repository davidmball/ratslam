<img src='https://wiki.qut.edu.au/download/attachments/104094381/logo_sml.jpg?version=1&modificationDate=1338441816000'>

<b>RatSLAM</b> is a bio-inspired simultaneous localisation and mapping (SLAM) system. Based on continous attractor network dynamics, RatSLAM is capable of mapping by closing loops to correct odometry error.<br>
<br>
The original RatSLAM algorithm was designed and implemented on Pioneer robots by Michael Milford and Gordon Wyeth (see <a href='http://eprints.qut.edu.au/37593/1/c37593.pdf'>RatSLAM: a hippocampal model for simultaneous localization and mapping</a>).<br>
<br>
There is an openRatSLAM paper available in <a href='http://www.springerlink.com/openurl.asp?genre=article&id=doi:10.1007/s10514-012-9317-9'>Autonomous Robots</a>. This paper describes how openRatSLAM works in technical detail.  If you use the code we would appreciate cites please.<br>
<br>
The C++ RatSLAM implementation is currently being used to power the iRat robot when it is <a href='http://ratslam.itee.uq.edu.au/live.html'>online</a> and in recent ports of the <a href='http://itee.uq.edu.au/~ruth/Lingodroids.htm'>Lingodroids project</a> to use the iRat.<br>
<br>
There are now two versions of RatSLAM available, both based on the same code:<br>
<ul><li><b>(NEW)</b> <a href='https://github.com/davidmball/ratslam/blob/wiki/RatSLAMROS.md'>A ROS-based version</a>
</li><li><a href='https://github.com/davidmball/ratslam/blob/wiki/RatSLAMLibrary.md'>A C++ library</a></li></ul>

We provide three datasets for the ROS version of openRatSLAM
<ul><li><a href='https://mega.nz/file/FAlXyZbB#6rMpQ6EE4LQIKmZvy5zN7Stdu4pIzZm2h3TnHkG2wms'>iRat in an Australian set (2011)</a><br>
</li><li><a href='https://mega.nz/file/od8xVbKJ#E81hKj-M1-CybBkX1dLe3htAJw-gP9MAQIEeZkPwuUY'>Car in St Lucia suburb (2007)</a><br>
</li><li><a href='https://mega.nz/file/oJdwxTAJ#EB-M_gLWq8Sy2uFvmER-D_uTZ7_Rd4v-5ZUhu1YGNCQ'>Oxford's New College (2008)</a></li></ul>

The code is released under the GNU GPL V3. Please contact David Ball if you require a more permissive license.
