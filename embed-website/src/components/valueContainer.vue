<template>
  <div class="p-3 m-3 row bg-light rounded-3" >
    <div class="col">
        <img class="rounded float-left mr-3" :src="require(`../assets/${imageUrl}`)" style="width: 100px" />
    </div>
    <div class="col">
      <h2 class="mt-0">{{ title }}</h2>
      <h2>{{ values[values.length-1] }}  {{ unit }}</h2>
    </div>
    <LineChart 
      :chartData="testData" 
      :width="50"
      :height="100"
    />
  </div>
</template>

<script>

import  { LineChart } from 'vue-chart-3';
import { Chart, registerables } from "chart.js";

Chart.register(...registerables);

export default {
  components:{
    LineChart
  },
  // data () {
  //   return {
  //     somevar :this.values
  //   }
  // },
  props: {
    imageUrl: {
      type: String,
      required: true
    },
    title: {
      type: String,
      required: true
    },
    values:{
      type: Array,
      required: true
    },
    unit:{
      type:String,
      required:true
    }
  },
  mounted() {
    // console.log(this.values); // Access propData in component's methods
    this.testData = {
      labels: ['-4', '-3', '-2','-1','current'],
      datasets: [
        {
          label: this.title,
          data: this.values,
          backgroundColor: ['#77CEFF', '#0079AF', '#123E6B', '#97B0C4', '#A5C8ED'],
          tension: 0.3,
        },
      ],
    }

  }
}
</script>

<style scoped>
</style>
