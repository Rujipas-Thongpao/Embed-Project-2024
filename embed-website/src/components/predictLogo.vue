<template>
  <img :src="require(`../assets/weather_icons/${this.forecasts[this.z].imgPath}`)" />
  <h1> {{ this.forecasts[this.z].description }}</h1>
</template>

<script>
  import forecastData from '../assets/forecast.json'


  export default {
    data() {
      return {
        forecasts : forecastData
      }
    },
    props: {
      z: {
        type: Number,
        required: true
      },
    },
  }


</script>



