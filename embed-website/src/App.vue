<template>
  <!-- <img alt="Vue logo" src="./assets/weather.webp" /> -->
  <predictLogo :z = "this.z"/>
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
            unit="hPa"
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
    const colref = await collection(db, "data");
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
  margin: 60px;
}
</style>
