<template>
  <img alt="Vue logo" src="./assets/weather.webp" />
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



export default {
  name: "App",
  components:{
    valueContainer,
  },
  data () {
    return {
      temperatures: [],
      humidities:[],
      pressures:[],
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
      this.dataReady = true;
    });
    console.log(this.humidities)
  },

  setup() {
    const testData = {
      labels: ['Paris', 'Nîmes'],
      datasets: [
        {
          data: [30, 40],
          backgroundColor: ['#77CEFF', '#0079AF', '#123E6B', '#97B0C4', '#A5C8ED'],
        },
      ],
    }
    return { testData };
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
