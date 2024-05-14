<template>
  <img alt="Vue logo" src="../assets/weather.webp" />
</template>


<script>
  import forecastData from '../assets/forecast.json'
  export default {
    props: {
      z: {
        type: Number,
        required: true
      },
    },

  }
</script>



