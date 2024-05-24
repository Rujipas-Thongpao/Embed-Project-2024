<template>
  <nav class="navbar navbar-dark bg-primary p-2 shadow">
    <div class="navbar-brand ">
      <div class="row">
        <div class="col">
          <img src="./assets/weather-forecast.png" width=70px/>
        </div>
        <div class="col">
          <H2 class="text-white mt-3">weather forecast</H2>
        </div>
      </div>
    </div>
  </nav>
  <predictLogo :z = "this.z" />
  <div class="container">
    <div class="row ">
      <div class="col">
        <valueContainer
            imageUrl="temperature.png"
            title="Temperature"
            :values="this.temperatures"
            unit="°C"
        />
      </div>
      <div class="col">
        <valueContainer
            imageUrl="pressure.png"
            title="Pressure"
            :values="this.pressures"
            unit="Pa"
        />
      </div>
      <div class="col">
        <valueContainer
            imageUrl="humidity.png"
            title="Humidity"
            :values="this.humidities"
            unit="%"
        />
      </div>
    </div>

  </div>
</template>

<script>
import  valueContainer  from './components/valueContainer.vue';
import { collection, getDocs, query ,orderBy, limit} from 'firebase/firestore';
import {db} from './firebase.js'
import predictLogo from './components/predictLogo.vue';



export default {
  name: "App",
  components:{
    valueContainer,
    predictLogo
  },
  data () {
    return {
      temperatures: [],
      humidities:[],
      pressures:[],
      z:0,
      dataReadys: false
    }
  },
  async mounted(){
    const colref = await collection(db, "sensor_data");
    const q = query(colref, orderBy("date","desc"), limit(5));
    const querySnapshot = await getDocs(q);
    querySnapshot.forEach((doc) => {
      this.temperatures.push(doc.data().temperature);
      this.pressures.push(doc.data().pressure);
      this.humidities.push(doc.data().humidity);
      this.z = doc.data().z;
      this.dataReady = true;
    });
  },
};
</script>

<style>
#app {
  font-family: Avenir, Helvetica, Arial, sans-serif;
  text-align: center;
}
</style>
