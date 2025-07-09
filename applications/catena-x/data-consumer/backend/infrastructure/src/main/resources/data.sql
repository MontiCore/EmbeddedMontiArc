INSERT INTO public.policy (id, end_time, expires_on, local_logging, max_usages, remote_logging, start_time, target_id)
VALUES (1, '16:00:00', '2023-06-24', true, 100, false, '08:00:00', null);
INSERT INTO public.policy (id, end_time, expires_on, local_logging, max_usages, remote_logging, start_time, target_id)
VALUES (2, null, '2023-07-05', true, 1000, true, null, null);
INSERT INTO public.policy (id, end_time, expires_on, local_logging, max_usages, remote_logging, start_time, target_id)
VALUES (3, null, '2023-06-20', true, 2, false, null, null);


INSERT INTO public.metadata (id, description, logging_url, price, provider, title, policy_id)
VALUES (1,
        'This dataset contains 1000 data points from trucks driving through Europe. It can be used to train machine learning models',
        'logging', 19.99, 'Carrier GmbH', 'Benelux Truckdata', 1);
INSERT INTO public.metadata (id, description, logging_url, price, provider, title, policy_id)
VALUES (2, 'This dataset contains 500 entries. It can be used for damage forecasting. ', 'logging', 5.5, 'Carrier GmbH',
        'Western Europe Truck Data', 2);
INSERT INTO public.metadata (id, description, logging_url, price, provider, title, policy_id)
VALUES (3,
        'This small dataset can be used for testing your machine learning models. It contains a small sample of 100 entries. It is not recommended to use it for training your models.',
        'logging', 2.99, 'Carrier GmbH', 'ML Test Data Set', 3);


INSERT INTO public.offer (id, metadata_id)
VALUES ('f10d7f03-ae67-4c51-ba92-ab62f7962973', 1);
INSERT INTO public.offer (id, metadata_id)
VALUES ('6355f4d9-6361-4a55-89d8-57bcd5666532', 2);
INSERT INTO public.offer (id, metadata_id)
VALUES ('d9c48beb-24a6-4663-8e6e-a361ca74a114', 3);


INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611482441, '71919bfcd628df321666fc97aac1c23b501d195b096e38bdcc5b7ce98b9bdd34', '2021-08-18 07:36:27.000000',
        13, 50.672425, 3.234842, 358850, 0, '2021-08-18 07:36:36.106000', 0, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611538045, 'deaa5ddce8433b459bd2e58cea13013907ce0c36d97894925271fc4093bafb6b', '2021-08-18 11:31:51.000000',
        23, 53.017757, 10.739292, 114549, 67, '2021-08-18 11:31:53.094000', 0, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611566774, '23b61e04f2b79148c317e349465c73e4b8dd270fb36c2f954322233af9cd0bdd', '2021-08-18 13:30:46.000000',
        29, 50.68277, 2.90924, -1, 13, '2021-08-18 13:30:48.456000', 0, null, 'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611477727, 'b633f9f4b423c2b7583a71feb9e285b63849016caf39ab3a547a63be09f116c5', '2021-08-18 07:16:54.000000',
        31, 50.6545, 3.1302, 880796, 80, '2021-08-18 07:16:57.054000', 8636, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611537883, '924eb069746f9eb6fd18064a3f30f9daf0f345632f60e0c1ceaceb5d45dcada6', '2021-08-18 11:31:07.000000', 5,
        41.29452, 1.36612, 390182, 82, '2021-08-18 11:31:09.676000', 0, null, 'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611482439, 'e0939a55640bf2b5503f50ad8d10e7619d4d683fa1719d4010f02fa726d90cf0', '2021-08-18 07:36:38.000000', 1,
        48.8562, 2.6645, 681969, 88, '2021-08-18 07:36:35.865000', 335960, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611499384, 'da2fed3132a2168afe2e13446c5e15ade8cfb6f08ba2190a63550c86eaa74f9c', '2021-08-18 08:39:32.000000', 6,
        50.8867, 4.0886, 1028153, 11, '2021-08-18 08:45:36.692000', 790474, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607951210, 'f42c60440c22300bc1a17ff98a4edd6a4603eeda61e2fa2a2664f87d9d52e024', '2021-07-28 11:16:35.000000', 7,
        48.2771, 3.8568, 330750, 80, '2021-07-28 11:16:36.102000', 60931, null, 'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611539056, 'bcbfe81f9ff07bdb9db33b07ac3e3507cd376e456e1d9ddcedd50ca3c76ceb7f', '2021-08-18 11:36:40.000000', 5,
        51.16407, 4.24272, 653455, 88, '2021-08-18 11:36:41.760000', 0, null, 'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607955396, '69d1beea1a60ff54825e94201a9f9e3ed70303cb9397fbe645a19feaf40d4205', '2021-07-28 11:32:14.000000',
        13, 47.2283, 5.7642, 40025, 0, '2021-07-28 11:32:13.417000', 23275, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611537802, 'c581db098f42f731b4333a3b4b64e303219c3464465ba3f64eded9abe3a58bdc', '2021-08-18 11:30:40.000000',
        19, 50.8869, 4.0889, 342341, 0, '2021-08-18 11:30:41.079000', -1, null, 'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611539548, 'cbb020eaf9b47937e770c307448de3c14c21752d230a998d72a7a2b105d86aab', '2021-08-18 11:38:49.000000', 8,
        48.509, 3.5939, 571788, 0, '2021-08-18 11:38:51.561000', 382089, null, 'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607957275, '512e79adcd66abb31651eac17a84c2622b056a6e0144cacf1664501a8c08c4d7', '2021-07-28 11:38:46.000000',
        18, 50.378, 3.6134, 208318, 5, '2021-07-28 11:39:01.037000', 114445, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611591283, '55660725b2a5f4a322fd312cd5f4310567ca80c7ab0c3cef79ae2f757a8cc1ac', '2021-08-18 15:17:08.000000',
        13, 50.3187, 3.1304, 839400, 82, '2021-08-18 15:17:09.707000', 419193, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611477912, '72b39502e859ff845fcdab4125271c169eb2b2df285aa86eecc66ae43233410d', '2021-08-18 07:17:50.000000', 1,
        50.452, 2.9777, 102433, 42, '2021-08-18 07:17:52.300000', -1, null, 'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611679990, '50a05605a1263a90ef656f72dd87964182330dbe91fdb8d9385bd124359c38f8', '2021-08-19 05:31:45.000000',
        22, 50.3721, 3.6093, 471404, 0, '2021-08-19 05:31:45.462000', 227643, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611567527, '73e886f393124af261d2bb257b5c941f2afe61d24c6d29936eae3d77ac22a3c0', '2021-08-18 13:34:10.000000',
        26, 51.4603, 7.8098, 169987, 49, '2021-08-18 13:34:11.257000', -1, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611484663, '4f2c3cea05c9d270f07d4d37f40dc1c551e5b4a9f6ab2d491f87ba419140342e', '2021-08-18 07:45:33.000000',
        30, 48.2718, 4.2066, 176586, 76, '2021-08-18 07:45:34.363000', 91132, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611490259, 'b252a2f1e8881963dd9a6df761c80358a9a69143fec7cb85e1eb18f14cab2abf', '2021-08-18 08:07:48.000000',
        17, 52.57986, 8.10955, 339726, 76, '2021-08-18 08:07:50.741000', 0, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611620993, '9f8bec23156f9234e9075da2818a05a6c63374a37afc12d81fed3a7fd48f9594', '2021-08-18 18:21:26.000000',
        21, 51.8724, 5.7283, 28185, 89, '2021-08-18 18:21:26.077000', 13647, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611600677, 'c581db098f42f731b4333a3b4b64e303219c3464465ba3f64eded9abe3a58bdc', '2021-08-18 16:02:38.000000', 1,
        50.9233, 4.7206, 342446, 48, '2021-08-18 16:02:39.188000', -1, null, 'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611567725, '65fb6d5c60fc04966f5da863fd1aa0611b0d8519b69bc42f573c342615ed21dc', '2021-08-18 13:34:59.000000',
        27, 51.0341, 2.846297, 613414, 45, '2021-08-18 13:35:01.043000', 0, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611476810, '854ba69cec35f6f3a7f04eb1b9cc091504d6e0119d998583104f49f3ed55ee27', '2021-08-18 07:13:11.000000', 9,
        50.6169, 3.0409, 497211, 66, '2021-08-18 07:13:13.888000', 20642, null, 'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611653426, 'c39f263f203d4097f668dba6a5c9077ea74e8e8bacb1e5165f2a5a76b2650889', '2021-08-19 02:35:15.000000',
        28, 47.8987, 3.3624, 881564, 89, '2021-08-19 02:35:16.590000', 8984, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607964145, '0ebf25fb07d3a52966cbdc694c16344ed4931f900a1f975322afd8241237b40c', '2021-07-28 12:04:20.000000', 0,
        50.9586, 2.9457, 43847, 71, '2021-07-28 12:04:18.570000', 32789, null, 'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611621980, 'ef4504074b5abc4db4bb412d89224d74ec721716f90334883bc9dd24d80b9701', '2021-08-18 18:31:34.000000',
        23, 50.3119, 2.8308, 451651, 80, '2021-08-18 18:31:35.600000', 224709, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611500336, 'b465304b5f1c8351c8c99b1dc7db84aee35f45843090f908a30bb9b4350f45c1', '2021-08-18 08:49:16.000000',
        34, 50.4456, 2.9745, 66381, 5, '2021-08-18 08:49:33.769000', 40856, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611591289, '9a6b07fd366a055530e78792821beb27a4f023374b3f20be486e8cc7d8caa7fe', '2021-08-18 15:17:13.000000',
        19, 45.2872, 4.822, 466117, 89, '2021-08-18 15:17:12.482000', 256686, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611481683, '1ede8c3bfdb078ecb53ee9896b121c2e3d8e5e4cee06f43907a065e891a9a571', '2021-08-18 07:33:25.000000', 6,
        50.856918, 3.399845, 262826, 90, '2021-08-18 07:33:32.246000', 0, null, 'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611634132, 'f691418165de3e61023cb9e5c20a13d506b9f24b53fb50c72aa37776681d5e41', '2021-08-18 21:15:52.000000', 7,
        51.05531, 4.51383, 267888, 0, '2021-08-18 21:19:52.060000', 0, null, 'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611499169, 'e07e98d08683f3c721547912e0af4899875487afeefcf942fadbee6f8ad1e9b9', '2021-08-18 08:44:35.000000',
        27, 50.4031, 3.0531, 193290, 79, '2021-08-18 08:44:34.546000', 99953, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611637844, 'ceb5874e488e2f3b44339df4660df51ec843f6c462b1ff9ec319847f0370b1af', '2021-08-18 22:25:01.000000',
        35, 49.082, 2.5514, 435472, 90, '2021-08-18 22:25:00.519000', 233218, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607958621, 'ce9895bdfa929cf9bebc04213c71cdfc187fb519c22c267e84381dbfdefec2ad', '2021-07-28 11:43:45.000000', 0,
        50.869, 2.894, 217, 0, '2021-07-28 11:43:43.151000', -1, null, 'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611632587, 'fdbcd7dd6f616c47b54965d2ced5feaebc49b458fe8afd09d8cbf65608657dc2', '2021-08-18 20:50:56.000000',
        26, 48.2065, 1.8524, 525405, 4, '2021-08-18 20:51:59.676000', 265316, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611528102, 'b633f9f4b423c2b7583a71feb9e285b63849016caf39ab3a547a63be09f116c5', '2021-08-18 10:48:11.000000',
        21, 50.3288, 2.9158, 880869, 80, '2021-08-18 10:48:13.223000', 8668, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611633926, '5d624d101b1b4892cb5d4da65330e7c5e7c401109afc1cc651ff7e3241fef7eb', '2021-08-18 21:16:24.000000', 0,
        49.4386, 2.6963, 690257, 89, '2021-08-18 21:16:23.647000', -1, null, 'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611508330, '31b0829fb3013180ef1e308cbdfc4023f6c43efa797f245874eb45bcbe424fa6', '2021-08-18 09:21:26.000000',
        19, 52.253525, 5.248535, 170369, 94, '2021-08-18 09:21:29.597000', 0, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611490074, '8b63fbba69f8ac28642f3b76270c135c1cc0728554d43cb24d73b6b1c3ce9fab', '2021-08-18 08:07:01.000000',
        21, 47.4603, 9.4326, 969520, 0, '2021-08-18 08:07:03.280000', 531553, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607951825, 'd8d7d8e8d88b217055be015c16347320f3e20b61cd1de8a542e4656287f2115d', '2021-07-28 11:19:16.000000',
        20, 47.4585, 5.1312, 242957, 83, '2021-07-28 11:19:16.496000', 141034, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611494471, 'f837c779a69dc928669ed6b78c204694d2a6b138e5127de381fd0c022434ea20', '2021-08-18 08:25:07.000000', 8,
        50.8533, 2.7567, 28397, 49, '2021-08-18 08:25:10.391000', 1400, null, 'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611623349, '983766469d6f7a033fd49c017c3beb0f2ae0658888adaa9f27a945962bc67db7', '2021-08-18 18:43:51.000000',
        20, 50.4806, 2.9814, 283126, 90, '2021-08-18 18:45:22.565000', 166154, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607910813, 'e38a5fdc6fc13226221d9269628a5ad0dd55121551b57b04482798858caaa1b7', '2021-07-28 08:50:00.000000',
        13, 50.211622, 8.604149, 508556, 2, '2021-07-28 08:51:09.105000', 0, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611591239, 'f1136eab4d12d03deaf7f3cc5d30717d3896738cdf5375b7345c02c43575ab71', '2021-08-18 15:16:55.000000',
        24, 52.1767, 5.6481, 279986, 83, '2021-08-18 15:16:56.745000', 149855, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611503451, '688ec7dec27e6a17a0586743126700b816c1e72d47281ff09cce079db9ae0845', '2021-08-18 09:02:02.000000',
        23, 48.2447, 5.3094, 243045, 48, '2021-08-18 09:02:03.020000', 143613, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607958828, '40ff149a2e0173fee73ec4354e74541167eacdb792563ecbb5a711bb4c9e8fca', '2021-07-28 11:44:26.000000',
        32, 51.0013, 6.8855, 21746, 80, '2021-07-28 11:44:24.795000', 15065, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611634021, '285ea4ef1ae2964b59ea2d5fd3c2e76843238c69aaa0ed3b933f8bd29d881c01', '2021-08-18 21:18:14.000000', 3,
        50.869, 2.894, 3, 0, '2021-08-18 21:18:13.755000', -1, null, 'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611653249, '7f4758e8ed4d28033a9fb96b18a679c99bd87c25da877bb7bfb7a76450ec9cf1', '2021-08-19 02:32:52.000000', 0,
        51.020116, 3.033331, 269373, 39, '2021-08-19 02:33:06.195000', 0, null, 'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611611293, '18445be7450e187baa8105091ffe2867761b5a2d341784a42dcae7a28dbfb4c1', '2021-08-18 17:03:56.000000',
        33, 48.3865, 3.9541, 511171, 67, '2021-08-18 17:03:57.172000', 297337, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611539402, 'a11ce6a567628641ecc8b19595216515819b6edfee7e7d186a0a662f798898b3', '2021-08-18 11:38:04.000000',
        12, 51.083728, 3.427508, 372939, 89, '2021-08-18 11:38:08.953000', 0, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611638136, '10bb914a567debc59680cee0312f30522659de338a665ca6d06adf05115df26b', '2021-08-18 22:30:53.000000', 2,
        50.2599, 2.8626, 580960, 88, '2021-08-18 22:30:55.100000', 196608, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611639962, 'afad8ffabece80a1e127fcf71292b185c8c0bb01b1c81f5b9a84c33be2eaed35', '2021-08-18 23:03:17.000000', 1,
        50.8693, 2.8943, 364, 0, '2021-08-18 23:03:17.599000', -1, null, 'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611539122, 'f135c8cc8f11f84e013d88204ed8da970382451882ba3c7c62702b5b7ff07e7a', '2021-08-18 11:36:53.000000',
        28, 51.1039, 5.0388, 296016, 85, '2021-08-18 11:36:55.238000', 164355, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607957276, '512e79adcd66abb31651eac17a84c2622b056a6e0144cacf1664501a8c08c4d7', '2021-07-28 11:38:47.000000',
        18, 50.378, 3.6134, 208318, 6, '2021-07-28 11:39:01.130000', 114445, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611600653, 'ce3da3881d7ec905b141000c33fc7b45c8810c0504b740894021d54634f1cb5e', '2021-08-18 16:02:28.000000', 3,
        47.2229, 5.0092, 227274, 85, '2021-08-18 16:02:28.234000', 124775, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611658745, '726e34e19af1adec5032108f9fd6d0443c15b6a699c477be40b97c2a43230b63', '2021-08-19 03:29:15.000000',
        30, 48.5645, 2.4768, 488069, 90, '2021-08-19 03:29:17.172000', 243296, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611537923, 'd0b7a010201358026f028983e8987e8f76f0ad13fb8e0828f4f8eedfa6a9bc06', '2021-08-18 11:31:21.000000',
        13, 50.6626, 2.8882, 532637, 89, '2021-08-18 11:31:21.917000', -1, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611484752, '492950ae71662fe89e1818da06aa58e3f2636c39e1aa67fccea91741f5c9a9a2', '2021-08-18 07:46:04.000000',
        25, 50.8559, 2.7549, 853077, 0, '2021-08-18 07:46:02.008000', 587172, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611490147, 'a5a7851d0f93ed76b6c85f0680ffc5f58bcd7319ac7d20f06329a0bf33850095', '2021-08-18 08:07:25.000000',
        16, 50.3863, 3.1297, 694133, 83, '2021-08-18 08:07:24.422000', 367872, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611532121, 'da2fed3132a2168afe2e13446c5e15ade8cfb6f08ba2190a63550c86eaa74f9c', '2021-08-18 11:05:47.000000',
        10, 50.8832, 4.1608, 1028228, 84, '2021-08-18 11:05:49.466000', 790533, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611621567, 'a2317f2e57964acb279578d1b8aa791161259eefe0e4cf5d5fad4cb0766e41bc', '2021-08-18 18:27:18.000000',
        23, 50.79457, 3.23988, 118849, 88, '2021-08-18 18:27:19.126000', 0, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611544358, '1845ff5c5a67bdcd7dc51c957bec7f68c1da3c312cb5b27d7d657b4ab6eda321', '2021-08-18 11:59:41.000000',
        22, 50.36719, 3.12456, -1, 0, '2021-08-18 11:59:44.135000', 0, null, 'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611548801, 'f220e49e1c3a84461eb2db41a8bd76c8c8433fb4933c7a3ed2797bb39a4249d1', '2021-08-18 12:18:07.000000',
        11, 50.4067, 4.4148, 483166, 0, '2021-08-18 12:18:09.691000', 51233, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607910864, 'e5df443cae14c47054e9180b3bd4ddcf42b53ad831cad497acf3207a3c4e3485', '2021-07-28 08:51:21.000000',
        28, 50.4502, 1.6141, 605934, 0, '2021-07-28 08:51:20.377000', 320577, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607954848, 'e56d550ed7b1e96d79011441208576d752524bfe326e80fad3c91b856b236317', '2021-07-28 11:30:12.000000',
        20, 53.5315, 10.1374, 124375, 37, '2021-07-28 11:30:11.611000', 62052, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611480687, 'f0ddaf068fcb477673a9ecfa5f271d8789f9b7c16b855b2534b73df413247386', '2021-08-18 07:29:52.000000',
        33, 51.0958, 2.7819, 253491, 1, '2021-08-18 07:29:52.651000', 191945, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607952338, '673b4c5d794626b491688b585db3fd4d23e9176c30c0626afa191fccad15ca6d', '2021-07-28 11:21:12.000000', 0,
        50.4021, 2.9835, 358848, 90, '2021-07-28 11:21:12.149000', 235513, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611484324, '939196fa0e8a754c62006f831b437adb4e4c3caba22b25dd73315d4ad4599971', '2021-08-18 07:44:12.000000',
        13, 47.3891, 8.6185, 471433, 47, '2021-08-18 07:44:13.612000', 247716, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611539127, '928e49e0ee5de0a83a02dd3105227a4bec712eb685230d7d0950ff6a9ce93c6c', '2021-08-18 11:36:53.000000',
        14, 50.13302, 3.14468, 26807, 85, '2021-08-18 11:36:56.215000', 0, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611621707, '7dbeac237b9c21cfcf8e2fa8885eed66e2322202268bbeb8ad210375e3d1db02', '2021-08-18 18:28:53.000000',
        23, 51.2777, 4.2527, 248876, 0, '2021-08-18 18:28:54.056000', 146231, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611482250, '6e8bf60b0e7400fd1042b76f378903bd832b2d16d1fc7f561a1642a97d360644', '2021-08-18 07:35:45.000000',
        11, 50.61053, 5.61157, -1, 0, '2021-08-18 07:35:48.586000', 0, null, 'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607952770, '48ec46cfe15ca428e4a96d54375dc1a42867c3187316c5d0750c77de83b08335', '2021-07-28 11:22:44.000000',
        28, 45.29893, 8.35426, 49045, 76, '2021-07-28 11:22:47.751000', 0, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611580090, 'cbf02a081317f65e4cec735d0fb8c59da96855c46439f0afcaa3df9e34701c24', '2021-08-18 14:28:05.000000',
        18, 50.9569, 5.1164, 991149, 33, '2021-08-18 14:28:06.264000', 705152, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611657737, '5827890eaa12f62e20788e43bc603477eb6f3764c67e86d9a7cd17032cce4007', '2021-08-19 03:19:37.000000', 0,
        51.4441, 4.7383, 494162, 41, '2021-08-19 03:19:37.965000', 284430, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611611036, 'be34e23cd46f10b79b21624cf511ff6514804e350fe5ef1cd248bfc8fa2dc05c', '2021-08-18 17:00:48.000000', 0,
        45.14549, 9.07141, 181197, 71, '2021-08-18 17:02:22.442000', 0, null, 'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611493605, 'f283282b8f92755c05f9e5eebf23e9c41fc872abcc112f4f13807c1a264bb468', '2021-08-18 08:21:25.000000',
        20, 49.9949, 6.9265, 126979, 33, '2021-08-18 08:21:27.119000', 62151, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611482094, '939196fa0e8a754c62006f831b437adb4e4c3caba22b25dd73315d4ad4599971', '2021-08-18 07:35:15.000000',
        14, 47.4135, 8.635, 471429, 0, '2021-08-18 07:35:17.428000', 247712, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611538111, '320cc39e06bd72ac604aaf856dab4f45063889c476587475861bdc6f6ffb90fa', '2021-08-18 11:32:14.000000',
        28, 50.998, 5.481, 786090, 88, '2021-08-18 11:32:17.036000', 468996, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611634872, 'e465ac5a8c58a1463696d7f3d6ff970b314a99d5932c964f88f79da6d066b0b0', '2021-08-18 21:32:11.000000',
        10, 51.38072, 6.22852, 724093, 83, '2021-08-18 21:32:12.802000', 0, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611632197, 'ef4504074b5abc4db4bb412d89224d74ec721716f90334883bc9dd24d80b9701', '2021-08-18 20:45:41.000000',
        15, 49.5998, 1.2727, 451822, 89, '2021-08-18 20:45:43.961000', 224802, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611583833, '01b28910e52775ed88162107cbd6a500b269da59c6ee34380df92e849c3c13ad', '2021-08-18 14:43:58.000000',
        28, 46.2666, 5.0906, 362593, 88, '2021-08-18 14:44:00.527000', 204418, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611538012, 'a44b9efe74b21d638045d9490bf780310071816682e505742ddcc80c0a757777', '2021-08-18 11:31:39.000000',
        24, 50.9707, 3.191, 366830, 46, '2021-08-18 11:31:41.214000', 241538, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1608727402, 'f9b0eeb34aab8bb4e4d05ff6fb5a6a2e3d5ffa53d52c4558aac5f702f8d74f7e', '2021-08-02 06:53:21.000000', 3,
        50.727493, 2.514497, 55003, 27, '2021-08-02 06:53:26.819000', 0, null, 'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611490280, '9f8bec23156f9234e9075da2818a05a6c63374a37afc12d81fed3a7fd48f9594', '2021-08-18 08:05:58.000000',
        12, 51.3397, 6.6816, 27737, 0, '2021-08-18 08:07:59.728000', 13444, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611601729, '7165ef906fa3d65a3b65db674a5441d699a6a0d1f35eeec7e40fe6295780a0d2', '2021-08-18 16:08:24.000000', 0,
        50.869, 2.8941, 217, 0, '2021-08-18 16:08:22.987000', -1, null, 'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611621757, '8995699fdc56f8ba4c01d87200afd477458a7f7588c213531a1b7fde0ebf35e7', '2021-08-18 18:27:18.000000', 0,
        50.79497, 3.17905, -1, 0, '2021-08-18 18:29:21.928000', 0, null, 'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611490184, '3b1bfe3cb84fa02e3efbc15069cdfdf3422b0391e7d7a41e61c58913aa68d06b', '2021-08-18 08:07:28.000000',
        13, 50.22038, 3.17802, 505264, 25, '2021-08-18 08:07:30.238000', 0, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611489676, '51ba0c156e4866ef63ed016aa23fb6586b5cd7a48d204ec18e32806065186dcf', '2021-08-18 08:04:02.000000',
        11, 39.94748, -0.14433, 507131, 0, '2021-08-18 08:05:16.552000', 0, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611549683, '35331408097437c0763b066d88784b081ca3bbab46527cd912b31dddc56a020d', '2021-08-18 12:21:31.000000',
        24, 48.62969, 2.48337, 1020978, 81, '2021-08-18 12:21:33.559000', 0, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611512963, '4f8a22c4f5106f8af5efc24bad2a8f6a5f9ecc4c03400f4159394f17891c7b0f', '2021-08-18 09:41:19.000000',
        19, 51.12852, 4.37506, 300631, 1, '2021-08-18 09:41:21.448000', 0, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611521499, '72b39502e859ff845fcdab4125271c169eb2b2df285aa86eecc66ae43233410d', '2021-08-18 10:18:20.000000',
        31, 50.446, 2.9773, 102459, 25, '2021-08-18 10:18:21.267000', -1, null, 'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611520653, '5d624d101b1b4892cb5d4da65330e7c5e7c401109afc1cc651ff7e3241fef7eb', '2021-08-18 10:12:39.000000', 4,
        50.3846, 3.5309, 689839, 0, '2021-08-18 10:14:39.408000', -1, null, 'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611539614, 'f1136eab4d12d03deaf7f3cc5d30717d3896738cdf5375b7345c02c43575ab71', '2021-08-18 11:39:08.000000', 4,
        52.2342, 7.398, 279794, 70, '2021-08-18 11:39:12.142000', 149764, null, 'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607911286, 'a34125eaf3756f582e4960442378d719e913a2b96dd076e29b11ce53ebba16c0', '2021-07-28 08:53:01.000000',
        33, 50.08912, 3.23057, 45695, 0, '2021-07-28 08:53:06.388000', 0, null, 'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611566821, 'dbc66db876f7cde4affde7f125e60eb75ded4eb0148490b1cc16359795a6574f', '2021-08-18 13:30:59.000000',
        15, 51.013912, 3.037005, 813543, 0, '2021-08-18 13:31:02.617000', 0, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611648711, 'b1a1d6c26b5eed544e61fabdac99f1b68fd862bd13c53b595d47a949bdbc1e78', '2021-08-19 01:33:59.000000',
        31, 47.1982, 5.87, 706320, 0, '2021-08-19 01:33:59.516000', 365852, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611633785, '62934ee0dabda2d366e14dbc62035e72eb7cee9173381564e6db38ef85e8ce57', '2021-08-18 21:13:51.000000', 3,
        45.7392, 4.9852, 487638, 89, '2021-08-18 21:13:52.978000', 243101, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611648050, 'd000b8581647fc32293cdb4ca89054f97f62fa6592fb484b07181cd13fb9ae18', '2021-08-19 01:24:00.000000',
        35, 51.1184, 4.9014, 763634, 0, '2021-08-19 01:24:00.051000', 689522, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611658667, 'c44afc9978d3af159ffbdd8ecb079ac18cdc78b8789803b1b1524427eb1f619e', '2021-08-19 03:28:40.000000', 7,
        51.61037, 7.75044, 597524, 83, '2021-08-19 03:28:42.439000', 0, null, 'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611513798, '9f8bec23156f9234e9075da2818a05a6c63374a37afc12d81fed3a7fd48f9594', '2021-08-18 09:44:53.000000',
        34, 51.3306, 6.6768, 27741, 48, '2021-08-18 09:44:53.178000', 13446, null,
        'f10d7f03-ae67-4c51-ba92-ab62f7962973');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611620606, '38e3cdf925f53817fa97d61178e857aa1a5075f4df7946f0d75a13613069fc2c', '2021-08-18 18:16:57.000000',
        26, 51.193, 4.3139, 315193, 3, '2021-08-18 18:17:57.404000', 181271, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1608649135, '8b111f78d62638cec78849325f456e2dcbe7b40d39d289e93629ab87670dabe2', '2021-08-01 13:27:10.000000',
        31, 51.1514, 4.8735, 416614, 0, '2021-08-01 13:27:39.806000', 263743, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611481025, '7dfb4d53f13d8af53b94043311075a6fba38aa2ac2ead29f8f029136c49a2793', '2021-08-18 07:31:14.000000',
        31, 47.3461, 4.4597, 465093, 88, '2021-08-18 07:31:15.669000', 20302, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611626280, '19c360c49ddd049c5899c49746e5e9bdef281c0d4f96a40749fb1baaea312348', '2021-08-18 19:20:29.000000',
        25, 49.634, 1.1492, 711333, 90, '2021-08-18 19:20:32.716000', 473939, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611601227, '3073b6e13df4250f674ced4bbba3c55b4439a3f84632aca06c0b613e237a8c25', '2021-08-18 16:05:28.000000', 6,
        50.3178, 3.4174, 339290, 76, '2021-08-18 16:05:31.086000', 189447, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611633767, '43a2f9b798546317aa9257e4d3fcd1ff633e41aa02958f4a228d54821cbd3efd', '2021-08-18 21:13:25.000000',
        32, 49.1502, 8.1562, 600915, 89, '2021-08-18 21:13:24.782000', 365868, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611634246, '99d94b914ffcb8c937c56fcf762be2a517fa014df1b63a25c726f87f376ad17b', '2021-08-18 21:21:53.000000',
        12, 50.9789, 2.629, 253826, 0, '2021-08-18 21:21:53.823000', 184382, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611634857, 'b663048143e095be052d438e09b92cbef1a0ac16dc5bd2d9ebbb84a68f067994', '2021-08-18 21:32:00.000000',
        20, 50.0881, 3.2298, 233829, 0, '2021-08-18 21:32:03.238000', 128346, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611645132, '44f82b3913e74f1c530a1ff9721210802723b5d2f4739a6e6d01bd67bf40c136', '2021-08-19 00:37:02.000000',
        15, 50.869, 2.8941, 22, 0, '2021-08-19 00:37:02.212000', -1, null, 'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611638835, 'a2130fca8ee5a1779bcbcf78e6a08f8e5393a31452de2b1ce6ab8e65e3f8ff08', '2021-08-18 22:42:40.000000',
        15, 50.869, 2.8941, 22, 0, '2021-08-18 22:42:40.640000', -1, null, 'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611477041, '100ce180dcf3d5796ac2abed527170b7b4c9a07877cb91ad8742162726b54227', '2021-08-18 07:14:13.000000',
        17, 50.9862, 3.1764, 38366, 89, '2021-08-18 07:14:13.109000', 14550, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611511858, 'a05e672f92a07256f8bacfbc56e7bd75fb4830404cfd064db1a38f8cb97a8415', '2021-08-18 09:36:12.000000', 1,
        50.0852, 3.1998, 227672, 75, '2021-08-18 09:36:11.198000', 168586, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611492325, '933ca4540b0016310678e138f21cdf1c6a82472ee0047346a6b9224b2bb3ebee', '2021-08-18 08:15:08.000000',
        28, 51.0009, 7.9406, 285230, 0, '2021-08-18 08:16:11.410000', 171422, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611639916, 'afad8ffabece80a1e127fcf71292b185c8c0bb01b1c81f5b9a84c33be2eaed35', '2021-08-18 23:02:17.000000', 1,
        50.8694, 2.894, 364, 0, '2021-08-18 23:02:17.593000', -1, null, 'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611622099, '2a75ee0d0d0f8ee7514f365584821f82bb6ff494e316fbe9bfdd3c87f0070d54', '2021-08-18 18:30:56.000000',
        18, 46.4356, 4.8695, 39074, 87, '2021-08-18 18:32:55.117000', 22779, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611677000, '4de531aebbeb766842bed1da81004df43ef0ba9ff06e5d288bb5696d68b9e454', '2021-08-19 05:19:10.000000',
        34, 45.58528, 8.40499, 2956, 79, '2021-08-19 05:19:12.085000', 0, null, 'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611493598, '0872fa241d8c57628f073701d81ebd0756851b272aaaeb0bdb28950c3a5ef47e', '2021-08-18 08:21:23.000000',
        32, 50.4506, 3.0117, 64447, 1, '2021-08-18 08:21:24.481000', 38348, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611513743, 'ffc5e10c8641d4075e0de650cd5df697a2a171a4a9724d4dc63ae38dedaecdc4', '2021-08-18 09:44:26.000000',
        15, 50.9905, 3.2432, 384036, 0, '2021-08-18 09:44:38.047000', 69181, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611503777, '8abe33f13ca24de2825d5a60b43028425adfe078531afef4daa7445da2b65ee3', '2021-08-18 09:03:10.000000',
        22, 49.359372, 6.73703, 102036, 29, '2021-08-18 09:03:17.062000', 0, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611601869, '63968f446b8e6054d6c4398700b5bbddcc0f2797525a4625d764f4dff8369788', '2021-08-18 16:09:15.000000',
        15, 49.93517, 1.64171, 613624, 96, '2021-08-18 16:09:14.101000', 0, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611639984, '675b59cb4d7dddf3844153a01bd24ed29bdac06864bbd14f99fafbb184babf61', '2021-08-18 23:03:47.000000', 6,
        48.5182, 1.7936, 623854, 89, '2021-08-18 23:03:48.368000', -1, null, 'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611640020, 'dd269314535a453a31a59299b238d9731ccf381f92836a53b2d95b2c9ce9d837', '2021-08-18 23:04:26.000000',
        30, 50.9989, 6.5391, 1299410, 88, '2021-08-18 23:04:25.626000', 716318, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607958950, '80beb45434cb33c0a56e884ca5eae81feee4668f5944dd5674104f8c27c2084f', '2021-07-28 11:44:45.000000',
        18, 50.83524, 4.48386, 146144, 83, '2021-07-28 11:44:46.765000', 0, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611622011, '38e3cdf925f53817fa97d61178e857aa1a5075f4df7946f0d75a13613069fc2c', '2021-08-18 18:31:54.000000',
        35, 51.1938, 4.3143, 315193, 32, '2021-08-18 18:31:56.444000', 181271, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611513002, 'a8464443fa5b0e156bc90ad9eb1cfba4f31192f92dd78d152c69b06ec96bc53b', '2021-08-18 09:41:26.000000',
        18, 52.2424, 6.5698, 6073, 0, '2021-08-18 09:41:31.324000', 20100, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611533249, 'b733a7a306dfe844e2688c82f6f9eba7158ace94934501aa6e973c5eabcef583', '2021-08-18 11:10:34.000000',
        35, 50.904, 3.2214, 493976, 27, '2021-08-18 11:10:38.227000', 284342, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611638128, 'a05e672f92a07256f8bacfbc56e7bd75fb4830404cfd064db1a38f8cb97a8415', '2021-08-18 22:30:44.000000',
        20, 50.1994, 3.1935, 227895, 0, '2021-08-18 22:30:43.517000', 168769, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611513305, '4cbee97a20ec2552521c0ae22f8a0e47c02a0a671d0bda88afb6d04944109d53', '2021-08-18 09:42:43.000000',
        34, 51.0479, 4.4549, 58261, 82, '2021-08-18 09:42:45.547000', 33333, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611504923, 'e95069552fa78b7eedcb8617a5b9b18cc3998b3ceced2472de06570e6db3e572', '2021-08-18 09:06:58.000000',
        35, 53.501835, 9.9309, 17585, 26, '2021-08-18 09:07:35.943000', 0, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611623367, 'ceb5874e488e2f3b44339df4660df51ec843f6c462b1ff9ec319847f0370b1af', '2021-08-18 18:45:30.000000', 6,
        48.9608, 2.5428, 435297, 77, '2021-08-18 18:45:29.107000', 233141, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611635426, 'c55dd59de9a3d502fcc043e8f1297e11a1747c33f094a692fd943b7d48c70cc5', '2021-08-18 21:41:10.000000',
        19, 47.1369, 4.6999, 629287, 90, '2021-08-18 21:41:11.781000', 326332, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611622101, '2a75ee0d0d0f8ee7514f365584821f82bb6ff494e316fbe9bfdd3c87f0070d54', '2021-08-18 18:32:02.000000',
        19, 46.4227, 4.8665, 39076, 49, '2021-08-18 18:32:56.108000', 22780, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611653308, '0e7d7f3bc2f4f235472cc2f2c466487c668557399c62fc49da985895b9eff6b6', '2021-08-19 02:33:41.000000',
        16, 50.62625, 2.39144, 506805, 60, '2021-08-19 02:33:40.213000', 0, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611638292, '67d51505ada91af78afad5cd976327eb12e3b7eb7f379e971b643ac162a6d130', '2021-08-18 22:33:36.000000', 9,
        49.8843, 5.4165, 1327534, 86, '2021-08-18 22:33:35.008000', 713215, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611621363, 'f1f8a5f50fe9558d864abc0022662687d61444d197481048f81f7383dadbe898', '2021-08-18 18:25:16.000000',
        15, 48.9091, 2.6373, 759022, 80, '2021-08-18 18:25:19.663000', 154012, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607955069, 'ca5fed77ad1a4cf03f4ffef37e55601a304036bc6aed498e458798f3f8cec074', '2021-07-28 11:30:55.000000', 5,
        50.8196, 6.2199, 100986, 88, '2021-07-28 11:30:57.016000', 50922, null, 'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607951759, 'd8b4f5dfcfd35fea2dbeff4bae908d2235474ac1072217416440bcba57ad5641', '2021-07-28 11:18:53.000000',
        16, 51.252868, 4.235318, 341095, 0, '2021-07-28 11:18:57.895000', 0, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611657813, 'c9df1227dcf24d53ba71a94cdcfdef4470b1939e9348aa01f3f00f493ba3dc64', '2021-08-19 03:20:19.000000',
        11, 50.4005, 2.8278, 384198, 29, '2021-08-19 03:20:19.129000', 69310, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611653300, '188a4c930345897543dc358b903ce6b6f5ffbbd1a1604f90cda12a41a1052ad1', '2021-08-19 02:33:32.000000', 7,
        53.3182, 10.13198, 379708, 77, '2021-08-19 02:33:34.217000', 0, null, 'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611523400, '048aba894d649721a6af99deff453c29e6621045688b4340dce501b5aad2bfc7', '2021-08-18 10:27:00.000000',
        28, 51.1984, 4.3825, 372250, 40, '2021-08-18 10:26:59.200000', 213298, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611512073, '846dfa82f29ef12d8bd99bc3316ac14a4722fe743c90ea25d4d20628154ccadb', '2021-08-18 09:37:10.000000', 4,
        51.020177, 3.045648, -1, 0, '2021-08-18 09:37:13.779000', 0, null, 'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611490647, 'e12f74d010cc7e557a36f7ca453fd7a7b738627b7d00ef83209fba49131639c8', '2021-08-18 08:09:28.000000', 6,
        50.9387, 2.0792, 548546, 90, '2021-08-18 08:09:29.199000', 231905, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611503406, 'afad8ffabece80a1e127fcf71292b185c8c0bb01b1c81f5b9a84c33be2eaed35', '2021-08-18 09:01:51.000000',
        31, 50.8696, 2.894, 364, 0, '2021-08-18 09:01:52.362000', -1, null, 'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611497635, 'a181262138fa264b2f30a2ae5b6e8bf3063200c4b1071643660bd73de18f0bdf', '2021-08-18 08:37:53.000000', 4,
        53.412025, 11.129013, 114461, 72, '2021-08-18 08:38:05.840000', 0, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611638817, 'c581db098f42f731b4333a3b4b64e303219c3464465ba3f64eded9abe3a58bdc', '2021-08-18 22:42:15.000000',
        12, 51.0557, 4.8834, 342470, 0, '2021-08-18 22:42:16.588000', -1, null, 'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611513022, '882fbf1f66a75b4103412b29b9d1306b3c42982029a6119cc649f58967980498', '2021-08-18 09:41:34.000000',
        17, 50.3314, 3.4372, 754135, 0, '2021-08-18 09:41:36.825000', 141842, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611492444, '2680531799e303e3955b84954897286bec5fe2ba3e0f2896549e9b0074d7b25e', '2021-08-18 08:16:36.000000', 8,
        50.5148, 2.8469, 427828, 0, '2021-08-18 08:16:41.431000', 231155, null, 'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607951292, '76a2adda483dd2aa5b596a21119c9f09fbc859bda6b5aa259e627fcbb3b677bd', '2021-07-28 11:17:00.000000', 3,
        53.5298, 10.0243, 12428, 41, '2021-07-28 11:17:02.064000', -1, null, 'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607955074, '4bc330737fcdfeab3226a369a4d87f972374dbd66a40592ecf0a56291b2eba8b', '2021-07-28 11:30:55.000000',
        11, 48.4962, 3.4864, 249415, 0, '2021-07-28 11:30:58.266000', 136661, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611511162, '4445677c0930aa8b6949b583c0fe2a3936ae267bc08a60d3d2b118d11357eba4', '2021-08-18 09:33:06.000000', 0,
        49.351727, 6.723905, -1, 0, '2021-08-18 09:33:28.345000', 0, null, 'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611657852, 'c15c102aa1e9cf193a2f6f6a0e63df3847002578937ac895c8451cbd0fd18368', '2021-08-19 03:20:00.000000', 0,
        51.773159, 4.952108, 535728, 0, '2021-08-19 03:20:42.610000', 0, null, 'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611621846, '49fe023c2a83a94e0a9587b9ae33f9065073bf5a7d5439783a1d23cc2cffced0', '2021-08-18 18:30:09.000000',
        21, 50.3607, 2.9611, 572727, 89, '2021-08-18 18:30:10.299000', 349096, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611567206, 'd4be3ce2b974874f49869a6a1d7274bfbede9d837ab092fd9aface8120b7397e', '2021-08-18 13:32:45.000000', 4,
        51.252377, 4.25692, 358984, 27, '2021-08-18 13:32:47.529000', 0, null, 'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611638337, 'da21547f9c16625114be2387e732ba072e7613c8a6dfa4b74bd7d7380dfea7a8', '2021-08-18 22:34:23.000000',
        17, 50.1485, 7.5961, 774897, 91, '2021-08-18 22:34:22.769000', 429258, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611523723, 'ec4a7a4ed8ba1c3ff2f2783aee99d6058c64a46eecdb57862b3f35f2d4c25e87', '2021-08-18 10:28:26.000000', 3,
        47.1286, 4.971, 156095, 90, '2021-08-18 10:28:28.642000', 81628, null, 'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1608935434, 'ca7c4753ddcadb6462dfefeb3027d65561cb4b1811f5ca17d70ef8238af628db', '2021-08-03 04:48:26.000000',
        17, 50.26879, 3.32519, 776421, 7, '2021-08-03 04:48:29.167000', 0, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611635433, '292f70f858d8e9bfc641ce74819bba380ff6798931bd188e6a5e9328148182a1', '2021-08-18 21:41:14.000000',
        19, 50.796383, 2.962675, -1, 0, '2021-08-18 21:41:16.951000', 0, null, 'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611600655, '2c1d495584dd397a9b6a0a15f97a0aa9803e8b09c8d30de22ae016cd221d6e36', '2021-08-18 16:02:27.000000', 0,
        52.736018, 7.746048, 180661, 0, '2021-08-18 16:02:28.799000', 0, null, 'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611512640, 'f59722061202b289e5b032ade9cb31af7ea8706fddc1207bd6a9e8b40012a61e', '2021-08-18 09:37:45.000000',
        16, 50.8797, 2.8736, 472816, 0, '2021-08-18 09:39:49.652000', 50590, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611620531, '016fd69b6ece9c9b341b780c104169e710f7d80216ddfaee3ff71ab32b019525', '2021-08-18 18:17:18.000000',
        24, 51.7732, 4.953, 898311, 0, '2021-08-18 18:17:23.392000', 540042, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611648702, '7d69d89879b3840e4c939248632e2688f80afd960af6e7cecfc4f008d113c4a7', '2021-08-19 01:33:54.000000',
        14, 48.966, 8.4654, 775084, 49, '2021-08-19 01:33:54.297000', 429346, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611500965, 'ec4a7a4ed8ba1c3ff2f2783aee99d6058c64a46eecdb57862b3f35f2d4c25e87', '2021-08-18 08:52:04.000000',
        10, 46.4724, 4.0283, 155967, 78, '2021-08-18 08:52:06.904000', 81564, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611638364, 'c44476d5e811be3115e1675da971d1d753f16797f6b14dacb582ac54fae8c457', '2021-08-18 22:34:43.000000',
        26, 48.2209, 4.1332, 244177, 0, '2021-08-18 22:34:46.898000', 136723, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611503703, '5b947bbfbbe4eda4991a32b0a25a4c075ceff0c5775e3c75688d2833ba5efc45', '2021-08-18 09:02:59.000000',
        27, 48.1407, 5.0477, 145305, 66, '2021-08-18 09:02:59.456000', 37640, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611611527, '4451f0e6227dedc4431a7cd5ec02c47ec134695a7997522009ca3c3e407b1070', '2021-08-18 17:05:24.000000', 0,
        49.813213, 1.509915, 613641, 0, '2021-08-18 17:05:25.810000', 0, null, 'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611698686, '94754b3bfebdf93672eaa3be1931a9e9271c274abfd6237f7471f031c620aa90', '2021-08-19 06:52:10.000000', 3,
        51.2861, 4.2655, 144411, 0, '2021-08-19 06:52:11.852000', 36230, null, 'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611677025, '0650d01d3d4e06e6de0bfc76f387a9a41e4ec6824a3bb8b7ab5198010c0281c1', '2021-08-19 05:19:18.000000',
        30, 48.36, 3.981, 921626, 80, '2021-08-19 05:19:18.304000', 172963, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611533141, '9a6b07fd366a055530e78792821beb27a4f023374b3f20be486e8cc7d8caa7fe', '2021-08-18 11:10:13.000000',
        22, 47.5447, 4.3434, 465807, 74, '2021-08-18 11:10:12.764000', 256490, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611648715, 'bec14ce53afdd1f632e663514cc7389ec715f22ece65b3d4001f837fbcfd00ce', '2021-08-19 01:33:00.000000', 9,
        51.7729, 4.9524, 1299588, 0, '2021-08-19 01:34:02.098000', 716417, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1608727107, '0d7a5e42169819c940d6c26ca6389a87f273ee0cc22db130d6d71bc65dd52f72', '2021-08-02 06:52:22.000000',
        22, 50.08317, 3.03336, 35752, 85, '2021-08-02 06:52:24.174000', 0, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611580085, '49c7b5bb01d38baae7b74493ca85849d27d2f0eb67d22f07a7ba50263e71b656', '2021-08-18 14:28:02.000000',
        18, 51.2512, 4.4259, 421626, 85, '2021-08-18 14:28:04.843000', 266720, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607951532, 'ff5cc58815d44557e1f67c40df77a7470d019822df449c99ac63256c968e26bc', '2021-07-28 11:17:58.000000', 9,
        48.5593, 3.1571, 497192, 56, '2021-07-28 11:17:59.476000', 19534, null, 'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611638343, '3073b6e13df4250f674ced4bbba3c55b4439a3f84632aca06c0b613e237a8c25', '2021-08-18 22:34:30.000000', 4,
        50.333, 3.4452, 339407, 89, '2021-08-18 22:34:31.817000', 189514, null, 'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611539121, '7a2a24d45a0eb850b4256082f5bc4bd145d6225d4a5303f9eba11e76a7836d4a', '2021-08-18 11:36:53.000000',
        20, 50.4329, 2.9716, 100732, 0, '2021-08-18 11:36:55.137000', 56375, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611635281, 'ad07f549f4c6b4b8d563c812490760abcfd724b3e8428c49f003a230bb148e59', '2021-08-18 21:38:41.000000',
        20, 50.3712, 3.6102, 506593, 0, '2021-08-18 21:38:46.743000', 165236, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611485081, 'a370c684dd0f3af8108509b391a80412b7eb4ec829284bd3f1cbdf19cfa1d9a5', '2021-08-18 07:47:04.000000',
        16, 49.96768, 3.19591, 467373, 88, '2021-08-18 07:47:06.645000', 0, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611639870, '675b59cb4d7dddf3844153a01bd24ed29bdac06864bbd14f99fafbb184babf61', '2021-08-18 23:01:19.000000', 8,
        48.5098, 1.7461, 623850, 88, '2021-08-18 23:01:20.423000', -1, null, 'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607951533, '3a380827771eb2e5ef6763ab40798724a6b4b0c77d7cfa4a65864d3ea0684d33', '2021-07-28 11:17:58.000000',
        29, 51.2789, 4.2802, 276839, 54, '2021-07-28 11:17:59.760000', 166339, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611503560, 'ac98104bb511d4eefb32df7bfd052ecc5047a55ff61c0d54571051eeeaa43be9', '2021-08-18 09:02:27.000000', 3,
        50.2783, 3.3409, 471076, 89, '2021-08-18 09:02:28.480000', 227482, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611544884, 'f59722061202b289e5b032ade9cb31af7ea8706fddc1207bd6a9e8b40012a61e', '2021-08-18 12:01:51.000000',
        24, 50.8199, 2.8297, 472869, 32, '2021-08-18 12:01:54.211000', 50627, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611494474, '7aaf32a4c072350adc03145ed85e1a1a821286a0502bbe23dfa48ddc46dd5a9c', '2021-08-18 08:25:10.000000',
        19, 48.6162, 4.1692, 47009, 89, '2021-08-18 08:25:10.564000', 27958, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607957979, 'bd14b21236e9bbe736f3c730dd7795c98ced6f5e85a63470cb94a76e2390617b', '2021-07-28 11:41:27.000000',
        24, 51.0136, 3.0361, 571284, 5, '2021-07-28 11:41:30.132000', 370838, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611680664, '837f88bf28e86d4466fe82b15ffa162d701671a86122ab779a5c8bb6c838c521', '2021-08-19 05:35:12.000000', 4,
        50.86905, 2.89407, 32, 0, '2021-08-19 05:35:06.104000', 0, null, 'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611509672, '978eb3b740f9521be2a374cbd68a33d3019676e8f4322e2a94b0882b2f49f9e6', '2021-08-18 09:27:22.000000',
        20, 50.3799, 3.6089, 449722, 89, '2021-08-18 09:27:23.725000', 203522, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611635919, 'f1f8a5f50fe9558d864abc0022662687d61444d197481048f81f7383dadbe898', '2021-08-18 21:49:14.000000',
        16, 48.0192, 2.6788, 759155, 90, '2021-08-18 21:49:15.476000', 154073, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611623866, '10bb914a567debc59680cee0312f30522659de338a665ca6d06adf05115df26b', '2021-08-18 18:51:10.000000',
        25, 48.6485, 2.5561, 580747, 0, '2021-08-18 18:51:12.021000', 196608, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611539691, 'da2fed3132a2168afe2e13446c5e15ade8cfb6f08ba2190a63550c86eaa74f9c', '2021-08-18 11:39:31.000000', 4,
        50.8962, 4.6109, 1028268, 49, '2021-08-18 11:39:33.346000', 790549, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611623908, '10bb914a567debc59680cee0312f30522659de338a665ca6d06adf05115df26b', '2021-08-18 18:51:16.000000',
        25, 48.6485, 2.5561, 580747, 5, '2021-08-18 18:51:39.624000', 196608, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611638310, 'afad8ffabece80a1e127fcf71292b185c8c0bb01b1c81f5b9a84c33be2eaed35', '2021-08-18 22:33:57.000000',
        18, 50.8681, 2.8934, 364, 0, '2021-08-18 22:33:58.342000', -1, null, 'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611676535, 'ecc6148c877ada35b72cca7331e2de6ef36d7fbce1b0dcb492ee42bcd5ccbfa6', '2021-08-19 05:16:36.000000', 9,
        48.5268, 4.1878, 149323, 0, '2021-08-19 05:16:54.820000', 85745, null, 'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611512942, '7e8b79241112a9a50aab9d0c095b29a19dc2266e467676c217326bf10c2792f7', '2021-08-18 09:41:13.000000',
        23, 50.45053, 3.01207, 29030, 4, '2021-08-18 09:41:16.055000', 0, null, 'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611508118, '5dc63ca8a62368aef128017daea9201f863b6464b0d506a6cffeb19ac41c3d22', '2021-08-18 09:20:28.000000',
        27, 50.39934, 3.10943, 242267, 93, '2021-08-18 09:20:30.853000', 0, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611478904, '6d50a347f1ac0d7664955ee3510774496b0f1a10adfc97c8a22f12870321ed37', '2021-08-18 07:22:24.000000',
        12, 50.4078, 2.9704, 596488, 34, '2021-08-18 07:22:25.685000', -1, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611503995, '4ccd3a9374b020734802c10c0fc4c956a0f40682529a239c6e3455cc45631513', '2021-08-18 09:04:13.000000', 0,
        48.2865, 4.0401, 250558, 88, '2021-08-18 09:04:13.294000', 145727, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611633790, 'a1ad85139997d53184804439688a85f0e2ca05dcbc5fb9a09edec8f3ec091e6b', '2021-08-18 21:13:56.000000',
        19, 50.1998, 3.1936, 250493, 0, '2021-08-18 21:13:57.105000', -1, null, 'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611621748, '0ff4ff2d85d4d5dc7dde1d4b93209888ff5f7510f82133624f52db54b3e13a8b', '2021-08-18 18:29:18.000000',
        30, 51.32521, 4.34515, 111942, 7, '2021-08-18 18:29:19.208000', 0, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611622166, '95e1ee4b9b3af35177fc2929e39ae17cc806626a54d3ff392f31e25d85802999', '2021-08-18 18:33:23.000000', 5,
        51.1473, 4.1964, 436552, 90, '2021-08-18 18:33:24.722000', 198249, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611624041, '10bb914a567debc59680cee0312f30522659de338a665ca6d06adf05115df26b', '2021-08-18 18:53:08.000000',
        35, 48.6475, 2.5584, 580747, 0, '2021-08-18 18:53:10.473000', 196608, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611620062, '8955816e11603a6f0f7633ee41825eb409af52c3eb0e167f17d8b87c940d17ac', '2021-08-18 18:13:08.000000',
        35, 50.3592, 5.6757, 295192, 0, '2021-08-18 18:13:10.201000', 166917, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611489700, '3a14403175269eb7e05ffbef10cd4027d252676a9528023554d68961f087b384', '2021-08-18 08:05:23.000000',
        25, 50.4331, 2.8615, 117495, 72, '2021-08-18 08:05:23.133000', 72341, null,
        'd9c48beb-24a6-4663-8e6e-a361ca74a114');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611539184, '221dbc0ba9190ad573ba968a54858ac5c7ebc3a310ccf2cb114c7b6dd9655a64', '2021-08-18 11:37:02.000000', 6,
        51.085152, 3.667262, 40780, 90, '2021-08-18 11:37:07.887000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611634278, '31b0829fb3013180ef1e308cbdfc4023f6c43efa797f245874eb45bcbe424fa6', '2021-08-18 21:22:23.000000',
        24, 53.15416, 6.31172, -1, 0, '2021-08-18 21:22:26.136000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611645124, 'ae55b118fa1c9dd1a95d24584f53646267d7c3a183dda26d8fcfd62234038fc9', '2021-08-19 00:36:54.000000',
        32, 51.5121, 5.4052, 101703, 92, '2021-08-19 00:36:52.487000', 52446, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607619712, 'a459dbb567be45996bfa27902f71d57884eae6dc70e29c89663d79ffa4fa151e', '2021-07-27 06:51:22.000000', 0,
        51.0575, 4.1227, 746265, 60, '2021-07-27 06:51:22.779000', 688349, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611676359, '4bba18a760b7d71dd817cb4896f61179192f40eee6c4517e423f03df1ffa2ae9', '2021-08-19 05:16:00.000000',
        24, 50.8885, 4.5443, 1028486, 6, '2021-08-19 05:16:01.522000', 790713, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611485077, 'd327ef1eba3a4ade469d98c5ca8cf5c45f45f3eaeabfc6beb0e65730a3947364', '2021-08-18 07:47:06.000000',
        13, 50.9548, 5.2996, 763456, 0, '2021-08-18 07:47:06.450000', 689346, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611484119, '100ce180dcf3d5796ac2abed527170b7b4c9a07877cb91ad8742162726b54227', '2021-08-18 07:28:08.000000',
        16, 50.9339, 3.251, 38377, 43, '2021-08-18 07:43:27.030000', 14554, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611602367, 'da2fed3132a2168afe2e13446c5e15ade8cfb6f08ba2190a63550c86eaa74f9c', '2021-08-18 16:12:08.000000',
        22, 50.7907, 5.1758, 1028402, 2, '2021-08-18 16:12:09.959000', 790646, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611676339, '8f84d72e61010945364f3699f75ff3e1c18d1827c0b998ff171bed716f2800fc', '2021-08-19 05:15:53.000000',
        10, 50.4476, 2.9774, 525781, 0, '2021-08-19 05:15:54.378000', 265476, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611513010, '01af937932a8996feac774197300879f22e5d93e1cf4103d270ec196185d2dba', '2021-08-18 09:41:32.000000',
        33, 46.5208, 3.3514, 1216561, 85, '2021-08-18 09:41:33.981000', 798851, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611488614, 'c9e33951a430606550e5ea65bc3a20b7972eae5962ff14bc176b68210bd14bcc', '2021-08-18 07:58:39.000000',
        34, 50.83205, 3.11523, -1, 63, '2021-08-18 08:00:36.427000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611528289, '72b39502e859ff845fcdab4125271c169eb2b2df285aa86eecc66ae43233410d', '2021-08-18 10:48:02.000000', 0,
        50.4506, 3.0116, 102467, 1, '2021-08-18 10:49:04.438000', -1, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611632170, '43a2f9b798546317aa9257e4d3fcd1ff633e41aa02958f4a228d54821cbd3efd', '2021-08-18 20:45:20.000000', 2,
        48.9085, 8.3492, 600877, 89, '2021-08-18 20:45:19.260000', 365848, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611558265, '347e5ecd764803586de4f9abee5186924dff081e0e3b4df97be44c583758caf9', '2021-08-18 12:54:41.000000', 0,
        49.6237, 2.7547, 648748, 0, '2021-08-18 12:55:44.124000', 283240, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611699177, '2fb802ace008b43a563201d8024f038c69db08d81133b4b8a25a27931ba9b17d', '2021-08-19 06:53:02.000000', 5,
        47.9351, 2.0023, 143138, 0, '2021-08-19 06:54:03.700000', 78444, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611646093, '11ab08e225e1979b5c1718f003fe9a9bea5a29622448ff1a79a0962564751699', '2021-08-19 00:53:51.000000',
        14, 52.0903, 0.1457, 466299, 89, '2021-08-19 00:53:50.329000', 240974, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611633846, 'f6a276da9f90bdeb881796f939022bda10be6b1aac347c74003f5b91f0583db9', '2021-08-18 21:14:32.000000',
        20, 51.5074, 7.4801, 549433, 5, '2021-08-18 21:14:55.834000', 349283, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611489707, 'bc37deaffd37b34fdeb72363b0a0d51bfba30403b88a9507c6b365ad27cc3de8', '2021-08-18 08:05:24.000000',
        21, 51.3012, 4.3014, 434108, 77, '2021-08-18 08:05:24.040000', -1, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611495254, 'c581db098f42f731b4333a3b4b64e303219c3464465ba3f64eded9abe3a58bdc', '2021-08-18 08:28:29.000000',
        35, 50.9062, 4.0325, 342260, 76, '2021-08-18 08:28:29.936000', -1, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611579755, '8685497da527097062851be2c8e7beeef4bff710ddcbf955594b635176741cac', '2021-08-18 14:26:31.000000', 2,
        47.6367, 1.3584, 685758, 90, '2021-08-18 14:26:31.563000', 364186, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611602362, '2606a198b7d60e8e3a845b857a1197837c458daf7284f37d9c09af03f398774f', '2021-08-18 16:11:08.000000', 2,
        50.473, 2.9585, 5207, 14, '2021-08-18 16:12:08.287000', 2766, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611539621, '5586fe3e16a44ba47ad2e1eca518c7cd2950282d59190708f5731abb2743da54', '2021-08-18 11:39:10.000000',
        17, 51.31277, 3.1876, -1, 46, '2021-08-18 11:39:14.533000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611549701, 'cb8b06828c80511f1c3a3955662373b303a77f0a9b0b0f9abc8c7af924b68718', '2021-08-18 12:21:34.000000',
        28, 50.9673, 8.146, 25899, 26, '2021-08-18 12:21:36.137000', 17929, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611623210, 'c598871a4dd9c98fc1b650b5cdd0baaf480ae7a90def021a90941e28eca586ea', '2021-08-18 18:43:44.000000', 7,
        51.718, 5.4348, 891383, 88, '2021-08-18 18:43:44.726000', 523742, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607964204, 'af2d7bd9eb1a79f3dd16217e3049de7fa6b9d9a60c77caa6bf921647a6be1dea', '2021-07-13 10:47:17.000000',
        255, 47.7126, 9.0973, 742, 0, '2021-07-28 12:04:31.672000', -1, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611513348, 'fdbcd7dd6f616c47b54965d2ced5feaebc49b458fe8afd09d8cbf65608657dc2', '2021-08-18 09:39:57.000000',
        31, 50.6255, 3.0243, 524930, 7, '2021-08-18 09:42:58.439000', 265097, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611638100, 'da21547f9c16625114be2387e732ba072e7613c8a6dfa4b74bd7d7380dfea7a8', '2021-08-18 22:30:06.000000',
        14, 50.202, 7.578, 774891, 87, '2021-08-18 22:30:06.380000', 429254, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611529664, 'd0a0300c5eff0fc7cada73f4ed02cf0e8910a0f781722633f9bce8d4058cafa7', '2021-08-18 10:55:02.000000',
        30, 50.766457, 3.277273, 40720, 89, '2021-08-18 10:55:07.587000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1608935861, '1e37a9456dc037ae42e0ab46cee270ec8962f773d21cf05d727c688b9573b32e', '2021-08-03 04:50:27.000000', 8,
        51.8736, 4.4027, 105727, 88, '2021-08-03 04:50:26.066000', 54766, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611538024, 'ac1b7e4e9be3d4e313d5bcc0e96a4c732604aca7d4222524874881fffbd231f0', '2021-08-18 11:31:43.000000', 8,
        45.44606, 10.85095, 199809, 50, '2021-08-18 11:31:46.044000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611657842, '31358c35b0f1d7da263502e7037478888f532de811f39d9cd911cc1dd2add27c', '2021-08-19 03:20:33.000000',
        22, 51.1049, 4.05, 467497, 89, '2021-08-19 03:20:34.731000', 278853, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611566812, '77286d0bf16a5c6bdfff5919812496dbc90be393615bba32730239ec4573d20f', '2021-08-18 13:31:02.000000',
        31, 46.8955, 6.3627, 500986, 50, '2021-08-18 13:31:01.203000', 308375, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611504918, 'd0b7a010201358026f028983e8987e8f76f0ad13fb8e0828f4f8eedfa6a9bc06', '2021-08-18 09:07:29.000000',
        29, 50.6944, 2.8079, 532487, 29, '2021-08-18 09:07:34.839000', -1, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611497704, '10bb914a567debc59680cee0312f30522659de338a665ca6d06adf05115df26b', '2021-08-18 08:37:22.000000',
        27, 50.3594, 3.0914, 580385, 0, '2021-08-18 08:38:23.824000', 196608, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611498833, 'd21e791f34ea55f671b6b199db5e60c8c385e57710aab19a82dc971f2c09828f', '2021-08-18 08:43:14.000000', 9,
        50.38136, 3.52578, 33849, 8, '2021-08-18 08:43:17.263000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611508125, '8972011485d692502d9e1281347fe94a5312e58676298e485416629fbbe187bf', '2021-08-18 09:20:30.000000',
        30, 50.446, 2.97387, -1, 0, '2021-08-18 09:20:32.488000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611510615, '7bcdb5c4da9af95282d368c051e9040a8f93ff1b894ff485047d7712a0a09f18', '2021-08-18 09:31:04.000000', 0,
        53.204028, 10.407432, 137110, 43, '2021-08-18 09:31:07.147000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611611568, '38e3cdf925f53817fa97d61178e857aa1a5075f4df7946f0d75a13613069fc2c', '2021-08-18 17:05:42.000000',
        29, 51.1924, 4.4022, 315184, 79, '2021-08-18 17:05:42.492000', 181267, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611703379, '5005752e8478642a1b306e5f18bd81521d688520d9022814b5e2bbf9a9eef904', '2021-08-19 07:11:28.000000',
        35, 53.11198, 6.206583, 173139, 0, '2021-08-19 07:11:30.628000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611638367, '734a8eb54f88ed21909b48065b1c67de9f2cbcb466398474f5c5611ef7aed52e', '2021-08-18 22:33:47.000000',
        26, 48.7918, 1.9656, 627759, 44, '2021-08-18 22:34:48.789000', 369817, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611639905, '97d2880a319adfb7edff4d467381d800aacdc1ecad444b8a6fc56882d40a8124', '2021-08-18 23:02:04.000000',
        31, 50.1467, 3.2044, 211377, 75, '2021-08-18 23:02:05.205000', 116495, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611633865, '0147e43ef72f8506049c642b6aaa3016d7db016690d6db3a71e161d6f4dd787c', '2021-08-18 21:15:15.000000',
        11, 51.3811, 5.89966, 724070, 82, '2021-08-18 21:15:17.019000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611538215, 'b8bd8df03741ea15b14662567a518b40b0681462f3470e2b0c11cdd647f82b41', '2021-08-18 11:32:00.000000', 0,
        51.773328, 4.952099, 570045, 0, '2021-08-18 11:32:46.271000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607958761, 'df8ecb8bbf355ce7a6fc49300b30082cea7dedd0c77a55aa3a232c9bfc60e983', '2021-07-28 11:44:12.000000',
        27, 51.2438, 4.7244, 702313, 90, '2021-07-28 11:44:15.297000', 439216, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611490284, '34f9f0b51ba7955dfa6608891f9852adde5310a87c00c06d8eb4fcb037d15d05', '2021-08-18 08:08:00.000000',
        26, 50.4287, 2.7901, 728308, 16, '2021-08-18 08:08:00.184000', 159757, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611539144, 'a44b9efe74b21d638045d9490bf780310071816682e505742ddcc80c0a757777', '2021-08-18 11:36:57.000000',
        31, 51.0133, 3.1651, 366836, 74, '2021-08-18 11:36:59.389000', 241544, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611511382, '4445677c0930aa8b6949b583c0fe2a3936ae267bc08a60d3d2b118d11357eba4', '2021-08-18 09:34:06.000000', 0,
        49.351727, 6.723893, -1, 0, '2021-08-18 09:34:25.502000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611583125, '608251a3fa3cc916d63f589b10bca5ee5f427f68130d75c8b6e869d37cd9d458', '2021-08-18 14:40:48.000000',
        14, 50.4457, 2.9746, 368664, 31, '2021-08-18 14:40:51.658000', 201011, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611558861, 'd9996891535ee83cb85fa51a33388adc12af4e65ddcc1854a8319e7436686302', '2021-08-18 12:58:10.000000',
        31, 49.1861, 4.1593, 681681, 83, '2021-08-18 12:58:11.162000', 407005, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611623220, 'c7014d6c2102e871c5cb25c05483417b883c0996d4ac4afb6a012ec2deaa3ece', '2021-08-18 18:43:50.000000', 2,
        46.9524, 3.1693, 818060, 79, '2021-08-18 18:43:50.773000', 265859, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611503759, '6bcee6607c2876e90205cc9192d72e8137a0c0d6b7a32c71d32c215f7f100d78', '2021-08-18 09:03:09.000000',
        34, 53.005458, 10.542045, 173618, 67, '2021-08-18 09:03:11.952000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611611710, 'e0939a55640bf2b5503f50ad8d10e7619d4d683fa1719d4010f02fa726d90cf0', '2021-08-18 17:06:38.000000',
        22, 50.3673, 3.1243, 682161, 0, '2021-08-18 17:06:39.318000', 336061, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611648002, '15f696e4e6533ed203279cefa0c61e1acc2f79d6cf322def26c13b9beb8a115c', '2021-08-19 01:23:22.000000',
        21, 50.86, 3.5792, 599772, 0, '2021-08-19 01:23:21.707000', 382857, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611632172, 'ad07f549f4c6b4b8d563c812490760abcfd724b3e8428c49f003a230bb148e59', '2021-08-18 20:45:21.000000',
        33, 50.3777, 3.4808, 506571, 47, '2021-08-18 20:45:21.525000', 165224, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611633868, 'a62e2dfb6f6bffa79902513bc8a5ebb562094ffc249520831f2fc0c7d40412f0', '2021-08-18 21:15:20.000000', 3,
        48.237, 4.4514, 226156, 0, '2021-08-18 21:15:20.954000', 10938, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611528360, '0b86fc9166911b4cddf39896806246268d908c3a608e9825edc3ea758c1922dd', '2021-08-18 10:49:19.000000',
        29, 50.8568, 5.1511, 19890, 0, '2021-08-18 10:49:26.208000', 14285, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611513447, '99d94b914ffcb8c937c56fcf762be2a517fa014df1b63a25c726f87f376ad17b', '2021-08-18 09:43:20.000000',
        13, 50.8949, 2.8634, 253665, 39, '2021-08-18 09:43:21.657000', 184257, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611490002, 'f28329118b999cbe3a3f17a2d381c2cdafafa0ec5abaa09c0e0adab7a4e63418', '2021-08-18 08:06:41.000000',
        33, 50.3714, 3.1107, 662712, 0, '2021-08-18 08:06:47.962000', 351615, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611639961, 'ab151fbdb69d5f52ec3318aa5f3ec3d33a957e819759b4725f322bc97a55a1be', '2021-08-18 23:03:13.000000',
        21, 51.02471, 5.81527, 204661, 84, '2021-08-18 23:03:16.661000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611499297, '6bcee6607c2876e90205cc9192d72e8137a0c0d6b7a32c71d32c215f7f100d78', '2021-08-18 08:45:08.000000',
        34, 52.865788, 10.675318, 173599, 63, '2021-08-18 08:45:16.209000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611531120, '3c1bacdc6613cb458577fc7f618d7bbaf5652a4c537d6b525dae81ccbb6ecda3', '2021-08-18 11:01:26.000000',
        19, 50.8204, 1.6972, 157860, 82, '2021-08-18 11:01:27.550000', 84687, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611490128, 'e465ac5a8c58a1463696d7f3d6ff970b314a99d5932c964f88f79da6d066b0b0', '2021-08-18 08:07:19.000000', 5,
        50.97541, 3.66298, 723545, 26, '2021-08-18 08:07:18.156000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611646101, '68e012e0ff18ba63fe7b13fac71749b51a75c2b36302cff8ecf87f2995cb4f30', '2021-08-19 00:53:55.000000',
        19, 51.1188, 4.9025, 76147, 5, '2021-08-19 00:53:57.018000', 41413, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607951228, '80beb45434cb33c0a56e884ca5eae81feee4668f5944dd5674104f8c27c2084f', '2021-07-28 11:16:40.000000',
        24, 50.9619, 4.81856, 146110, 54, '2021-07-28 11:16:41.516000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611635947, '956cff4b0fc01cb70ba12db1e679f1ecffef55a936c565ef562c8ac70cf0857d', '2021-08-18 21:48:55.000000', 5,
        48.7554, 2.3606, 582504, 17, '2021-08-18 21:49:55.122000', 359117, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611566716, 'f78fd3c4b2011b9cef9f25109dbda3e4c0ffb2ea09d634d92a783e836f66e716', '2021-08-18 13:30:28.000000', 4,
        50.2888, 3.357, 599308, 82, '2021-08-18 13:30:29.776000', 321960, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611623959, 'c7014d6c2102e871c5cb25c05483417b883c0996d4ac4afb6a012ec2deaa3ece', '2021-08-18 18:52:18.000000',
        29, 47.0272, 3.1672, 818071, 79, '2021-08-18 18:52:18.288000', 265867, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611638139, 'afad8ffabece80a1e127fcf71292b185c8c0bb01b1c81f5b9a84c33be2eaed35', '2021-08-18 22:30:57.000000',
        255, 50.8693, 2.8943, 364, 0, '2021-08-18 22:30:58.366000', -1, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611611180, '854ba69cec35f6f3a7f04eb1b9cc091504d6e0119d998583104f49f3ed55ee27', '2021-08-18 17:01:28.000000',
        12, 50.5974, 3.1428, 497400, 77, '2021-08-18 17:03:12.159000', 20642, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611497221, '18d1b352f08995965bad56439ca8a1aedb431c6caaa746ca9c29eae8de7db05e', '2021-08-18 08:35:22.000000',
        33, 51.1942, 4.3153, 587586, 0, '2021-08-18 08:36:22.094000', 328766, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611492457, '2680531799e303e3955b84954897286bec5fe2ba3e0f2896549e9b0074d7b25e', '2021-08-18 08:16:37.000000', 8,
        50.5148, 2.8469, 427828, 0, '2021-08-18 08:16:43.634000', 231155, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611601837, '1fedcb87147a644fb540852b3c71a1742e650e57b88f69f8a7eb56d27c3c37ec', '2021-08-18 16:09:05.000000',
        12, 47.9781, 0.2068, 613920, 0, '2021-08-18 16:09:05.952000', 325121, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611539687, '05c9e7bec0e010fa675378d0f6d10400182223904f9e4ea15e0d6dc82c7f66cd', '2021-08-18 11:39:29.000000',
        16, 52.892193, 10.653792, -1, 34, '2021-08-18 11:39:31.960000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611502675, 'fdcb9d1e2c8ae61b559afe278cfe2a854f77a5d247014994918a708f8a760956', '2021-08-18 08:58:44.000000', 0,
        50.8406, 2.8763, 905, 0, '2021-08-18 08:58:46.616000', -1, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611637813, '6b6f076451ab978dfd0690e973ec6801d11eb8b443ba7c1220ce9a76830ca9e0', '2021-08-18 22:24:20.000000',
        10, 48.1674, 4.4876, 706132, 89, '2021-08-18 22:24:18.629000', 365762, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611633715, 'a2334dd23f8de53f5af489aa17840ba9ad7f6900f6d176bdbe7cde48076a381f', '2021-08-18 21:12:10.000000', 1,
        45.7186, 4.97152, 255180, 86, '2021-08-18 21:12:11.859000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611508792, '55660725b2a5f4a322fd312cd5f4310567ca80c7ab0c3cef79ae2f757a8cc1ac', '2021-08-18 09:23:24.000000',
        28, 48.5537, 2.644, 839136, 39, '2021-08-18 09:23:25.783000', 419073, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611530256, 'afad8ffabece80a1e127fcf71292b185c8c0bb01b1c81f5b9a84c33be2eaed35', '2021-08-18 08:56:54.000000',
        22, 50.869, 2.8941, 364, 0, '2021-08-18 10:57:47.280000', -1, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611497192, '1df6a373db89e6f7339aa8a4dab499644f22fd1a77890375069a508d3f7299c6', '2021-08-18 08:36:07.000000',
        27, 51.0191, 3.1282, 711005, 60, '2021-08-18 08:36:10.489000', 473638, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611493446, '97d2880a319adfb7edff4d467381d800aacdc1ecad444b8a6fc56882d40a8124', '2021-08-18 08:20:45.000000', 2,
        50.0896, 3.2307, 211076, 0, '2021-08-18 08:20:50.404000', 116260, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611539166, '8abe33f13ca24de2825d5a60b43028425adfe078531afef4daa7445da2b65ee3', '2021-08-18 11:36:59.000000',
        15, 49.361047, 6.927933, 102086, 64, '2021-08-18 11:37:03.835000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611485030, '099a440b6e18a065dea9d6f9609ffc0e6f5d904e693f0bf0cc1f8371a9f5c16d', '2021-08-18 07:46:24.000000',
        20, 50.754212, 3.147933, 345007, 85, '2021-08-18 07:47:00.929000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611503561, '55660725b2a5f4a322fd312cd5f4310567ca80c7ab0c3cef79ae2f757a8cc1ac', '2021-08-18 09:00:05.000000', 3,
        48.4232, 2.7398, 839118, 0, '2021-08-18 09:02:28.715000', 419061, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611530306, 'ee6cb3f7129ef3b3889cc2cca2e9bb70b2402daccead5737d25a70bbe70e351c', '2021-08-18 10:57:58.000000',
        25, 50.976, 3.6257, 576388, 89, '2021-08-18 10:57:59.506000', 374496, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611512588, '6524e1d75675f91bc5eb3506f0b4f1713f1caf252b9c9839dfa80cd280ea22c2', '2021-08-18 09:39:32.000000',
        16, 46.94113, 5.55373, 145137, 79, '2021-08-18 09:39:34.133000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611528325, 'ffc5e10c8641d4075e0de650cd5df697a2a171a4a9724d4dc63ae38dedaecdc4', '2021-08-18 10:49:16.000000',
        28, 51.0141, 3.0369, 384054, 0, '2021-08-18 10:49:16.626000', 69193, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611500924, 'a181262138fa264b2f30a2ae5b6e8bf3063200c4b1071643660bd73de18f0bdf', '2021-08-18 08:51:54.000000',
        12, 53.419083, 11.188297, 114467, 0, '2021-08-18 08:51:56.427000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607955061, 'dedcf51b7ed9a64d7db9ac3e8ed35a5f6f053718b5dab956fc0cd7bbe5d5959d', '2021-07-28 11:29:41.000000',
        35, 51.00747, 2.18768, 430243, 0, '2021-07-28 11:30:55.366000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611633777, 'ad07f549f4c6b4b8d563c812490760abcfd724b3e8428c49f003a230bb148e59', '2021-08-18 21:13:46.000000',
        12, 50.3786, 3.4691, 506575, 46, '2021-08-18 21:13:46.850000', 165227, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611633814, '1d007e4d66843369d7f6d7350ecf975e0cb8dd29130d2ef5e110c79798ca2dfd', '2021-08-18 21:13:53.000000',
        13, 51.5765, 0.408, 404624, 5, '2021-08-18 21:14:18.138000', 213654, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611583086, '57f39a1e22442f0071a6d622c20beeda110329d37ceb85fa9dd4dd89b8ab831b', '2021-08-18 14:40:42.000000',
        32, 43.5899, 5.1944, 870807, 0, '2021-08-18 14:40:42.892000', 434380, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611633847, '62934ee0dabda2d366e14dbc62035e72eb7cee9173381564e6db38ef85e8ce57', '2021-08-18 21:14:55.000000',
        34, 45.7525, 4.9857, 487639, 89, '2021-08-18 21:14:56.960000', 243102, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611676947, '726e34e19af1adec5032108f9fd6d0443c15b6a699c477be40b97c2a43230b63', '2021-08-19 05:18:56.000000',
        35, 48.842, 2.6636, 488126, 81, '2021-08-19 05:18:58.386000', 243326, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611539972, 'ecac90eaca34202b3d2ea440fd52144108c59f1fe4a0e2a023cf1b90555adf04', '2021-08-18 11:40:31.000000',
        15, 51.526, 7.5179, 22925, 0, '2021-08-18 11:40:55.553000', 11791, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611539178, 'c581db098f42f731b4333a3b4b64e303219c3464465ba3f64eded9abe3a58bdc', '2021-08-18 11:37:06.000000',
        10, 50.8838, 4.1558, 342346, 88, '2021-08-18 11:37:06.817000', -1, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611626327, '393dc9f1bd7ee63f36984f7ccb9f055528b1ad978432e1178beb631b9cfdc843', '2021-08-18 19:21:06.000000',
        12, 51.7657, 5.1008, 539298, 70, '2021-08-18 19:21:05.816000', 317101, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607618029, '6eec4b11aa984c5a4381027abe641012b1b08b509203cfcdf015b174d29c911f', '2021-07-27 06:44:06.000000', 5,
        47.916, 8.4954, 96413, 8, '2021-07-27 06:45:05.045000', 48569, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611503558, '7dfb4d53f13d8af53b94043311075a6fba38aa2ac2ead29f8f029136c49a2793', '2021-08-18 08:55:23.000000',
        17, 47.9173, 3.5162, 465200, 0, '2021-08-18 09:02:28.527000', 20348, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611633933, '816a636e7dac5b64dec27bd3d39a4881cc443063d062dc9fe39d9686fae9f458', '2021-08-18 21:16:27.000000',
        35, 50.871788, 2.885185, 55357, 0, '2021-08-18 21:16:32.340000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611530524, '048aba894d649721a6af99deff453c29e6621045688b4340dce501b5aad2bfc7', '2021-08-18 10:58:53.000000',
        14, 51.193, 4.3149, 372257, 0, '2021-08-18 10:58:54.095000', 213305, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611498081, 'b23d4cb94f4e1aa8fcaeaff8b3983533e7d04a5dc8a6ba658f485a389c6e74c3', '2021-08-18 08:40:14.000000',
        18, 50.1569, 2.8732, 502740, 89, '2021-08-18 08:40:12.828000', 56404, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611647992, 'b1a1d6c26b5eed544e61fabdac99f1b68fd862bd13c53b595d47a949bdbc1e78', '2021-08-19 01:23:12.000000',
        33, 47.1971, 5.871, 706320, 12, '2021-08-19 01:23:11.435000', 365851, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611511813, '1ec49df62a5e78f4ec6c9d69a1d061c41d7ff995d54293dcba1e576e656a484d', '2021-08-18 09:36:03.000000',
        25, 51.01954, 3.2717, 506689, 3, '2021-08-18 09:36:03.493000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611537791, 'db6651e098793abaadbf4c0b52bff07d0f138a849d117449514d3d8227584eb7', '2021-08-18 11:30:37.000000',
        24, 51.0479, 3.8079, 81876, 10, '2021-08-18 11:30:37.829000', 46148, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611620881, '49fe023c2a83a94e0a9587b9ae33f9065073bf5a7d5439783a1d23cc2cffced0', '2021-08-18 18:20:28.000000',
        20, 50.4847, 2.9841, 572712, 91, '2021-08-18 18:20:29.182000', 349087, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611676960, 'ddfcc7aa156de43d43e88c1468df91bfa6f871a8560342c3251388d7f19197ad', '2021-08-19 05:18:38.000000', 7,
        39.66743, -0.256, 509878, 7, '2021-08-19 05:19:02.725000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611544318, 'dd4329949a025d3198ed4dac360ca828d1a7291963651b93f9990e471950390e', '2021-08-18 11:59:37.000000', 7,
        51.1695, 2.9743, 141755, 90, '2021-08-18 11:59:36.008000', 97008, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611512325, '10bb914a567debc59680cee0312f30522659de338a665ca6d06adf05115df26b', '2021-08-18 09:38:15.000000', 7,
        50.3697, 3.1243, 580395, 24, '2021-08-18 09:38:16.471000', 196608, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611511892, 'ea7ab228605f69b59c78da34fcbf96d9adbfa38c3ce67426ce7358987a1ef1c7', '2021-08-18 09:36:20.533000', 0,
        58.742683, 5.928213, -1, 0, '2021-08-18 09:36:21.044000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611698784, '80f1bbc72c2d58539ac9fa58cc684bca075bb8d521351c65d87739eb8f156177', '2021-08-19 06:52:34.000000',
        18, 50.7801, 7.2424, 28390, 57, '2021-08-19 06:52:34.232000', 13754, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607616913, 'bf15936ab2a9af177641929ef41ee666600dee3deea297045c0e41d228caf7ce', '2021-07-27 06:40:50.000000',
        32, 50.1962, 3.1976, 907758, 32, '2021-07-27 06:40:51.152000', 75095, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607951745, '6f450ffd2b18882c6f88a4a42400ff7fca44882d69d388fc2a4a54e32bf8b9a0', '2021-07-28 11:18:52.000000',
        29, 51.7733, 4.953, 491658, 0, '2021-07-28 11:18:53.645000', 303243, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611680147, '451d6e1de184e247a7a185300d8323cc5e40ca340e1832ac1b84947af971f451', '2021-08-19 05:32:22.000000',
        15, 50.8608, 1.7343, 786458, 0, '2021-08-19 05:32:31.403000', 469231, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611634886, 'f6a276da9f90bdeb881796f939022bda10be6b1aac347c74003f5b91f0583db9', '2021-08-18 21:27:37.000000',
        26, 51.4926, 7.3551, 549443, 70, '2021-08-18 21:32:16.235000', 349290, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611639959, '978eb3b740f9521be2a374cbd68a33d3019676e8f4322e2a94b0882b2f49f9e6', '2021-08-18 23:02:13.000000',
        30, 50.3676, 3.1256, 449778, 0, '2021-08-18 23:03:15.520000', 203522, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611483538, '362b618263f6b80b16b81cd2dbad0f380eb830d64518f147675eee215992e6ef', '2021-08-18 07:41:07.000000',
        15, 47.8637, 3.5389, 224102, 68, '2021-08-18 07:41:07.489000', -1, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611634093, 'ad07f549f4c6b4b8d563c812490760abcfd724b3e8428c49f003a230bb148e59', '2021-08-18 21:18:59.000000', 9,
        50.3371, 3.5151, 506582, 89, '2021-08-18 21:19:02.538000', 165231, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611634041, 'c5ea7a4141214b56a173f556265701b61a8eaf9ea2b48bd59933717aa147f4e7', '2021-08-18 21:18:29.000000', 2,
        48.237, 4.4511, 227677, 0, '2021-08-18 21:18:24.037000', 9514, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611548899, '853d4ade678024b335f0acd41189622a8ba8383fc074f5a1b1dbadb0d2877d48', '2021-08-18 12:18:29.000000',
        32, 46.4264, 4.412, 142822, 80, '2021-08-18 12:18:31.805000', 78290, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611676446, 'ca73990b5ffce4927c1dbf379b4e993f0499524141010138d4db0628bdf12c99', '2021-08-19 05:16:25.000000',
        23, 51.879055, 8.424577, 426571, 84, '2021-08-19 05:16:29.835000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611676417, '88cf689a18ed8eaef74ff7eb348656fa3ca932be4c33af553cf1c857fd322d06', '2021-08-19 05:16:01.000000',
        12, 50.4517, 3.0101, 102521, 5, '2021-08-19 05:16:20.556000', -1, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611639830, 'c1a318a517595c4e7383441f72fe94fec5cd2ac5572b94609fb3aed039837434', '2021-08-18 23:00:31.000000', 9,
        54.8532, 31.8616, -1, 0, '2021-08-18 23:00:29.914000', -1, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611523355, '42b20e9116f99dd9ceeb4aae435930d67adecc27d974446e62888ec23e4f4efb', '2021-08-18 10:25:34.000000', 3,
        45.23435, 9.0327, 1096, 0, '2021-08-18 10:26:46.833000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611528329, 'b633f9f4b423c2b7583a71feb9e285b63849016caf39ab3a547a63be09f116c5', '2021-08-18 10:49:15.000000',
        21, 50.3223, 2.9085, 880870, 71, '2021-08-18 10:49:17.720000', 8669, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611657879, 'b20190c0b3eb314d59e74aafb69973e4abca058bec3b64dcab85365d09ae940b', '2021-08-19 03:20:49.000000', 2,
        50.7526, 3.1471, 297847, 79, '2021-08-19 03:20:52.687000', 162357, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611498078, '933ca4540b0016310678e138f21cdf1c6a82472ee0047346a6b9224b2bb3ebee', '2021-08-18 08:39:08.000000',
        28, 51.0009, 7.9406, 285230, 0, '2021-08-18 08:40:11.713000', 171422, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611499158, '6d50a347f1ac0d7664955ee3510774496b0f1a10adfc97c8a22f12870321ed37', '2021-08-18 08:41:31.000000',
        13, 50.406, 2.9698, 596489, 3, '2021-08-18 08:44:32.453000', -1, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611513445, '2db440b6f10b234ee14aa89f4d70bac21265e463c896ff7c4663edee40fdc8f4', '2021-08-18 09:43:21.000000',
        29, 50.4376, 2.6615, 786324, 80, '2021-08-18 09:43:21.303000', 414691, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607952311, '3ddb2d3b622d6c6207466f194d50337c54cae34b922f36b786a2192303b769b0', '2021-07-28 11:19:40.000000', 1,
        50.2994, 3.8014, 769818, 0, '2021-07-28 11:21:07.368000', 143216, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611601582, 'cf437ddbad6970f5e8a1ba0dd9efd07e7ac6787c845fa50c2246eae1f6330e29', '2021-08-18 16:07:30.000000',
        20, 50.4457, 2.9746, 125451, 0, '2021-08-18 16:07:33.895000', -1, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611638869, '7165ef906fa3d65a3b65db674a5441d699a6a0d1f35eeec7e40fe6295780a0d2', '2021-08-18 22:43:14.000000', 0,
        50.869, 2.894, 217, 0, '2021-08-18 22:43:13.884000', -1, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611634408, '854ba69cec35f6f3a7f04eb1b9cc091504d6e0119d998583104f49f3ed55ee27', '2021-08-18 21:24:29.000000',
        20, 48.4359, 4.154, 497715, 90, '2021-08-18 21:24:31.839000', 20642, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611679999, '8fe72becc9fb7ff92b88fa81fabffd6244979f8d7fdecc72b6a4aba23907e444', '2021-08-19 05:31:44.000000',
        30, 48.2566, 4.2552, 340782, 51, '2021-08-19 05:31:47.691000', 194177, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607952318, 'd79a83973daf7ec6c69fb9473150dc677a8a4807ba7c37f3a659337eb05a5aff', '2021-07-28 11:21:07.000000', 4,
        50.863927, 2.859382, 207299, 65, '2021-07-28 11:21:08.664000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611633998, '43a2f9b798546317aa9257e4d3fcd1ff633e41aa02958f4a228d54821cbd3efd', '2021-08-18 21:17:43.000000',
        35, 49.205, 8.1479, 600922, 88, '2021-08-18 21:17:42.778000', 365870, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611520821, '65ce6e422d2f10260391cbec606621ed6a4dd8a3773a4e218ea751a2f607f491', '2021-08-18 10:15:30.000000', 8,
        49.853, 2.7421, 262207, 90, '2021-08-18 10:15:32.226000', 114173, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611490129, '0147e43ef72f8506049c642b6aaa3016d7db016690d6db3a71e161d6f4dd787c', '2021-08-18 08:07:19.000000', 5,
        50.97541, 3.66298, 723545, 26, '2021-08-18 08:07:18.198000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611523399, '9d644748d12075936583e9f9a435723cdb2713bf295fbd6f3e6c95f1e5e6bf71', '2021-08-18 10:26:59.000000', 5,
        51.0598, 2.6803, 172050, 89, '2021-08-18 10:26:59.038000', 123820, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611635892, 'b633f9f4b423c2b7583a71feb9e285b63849016caf39ab3a547a63be09f116c5', '2021-08-18 21:48:52.000000',
        13, 47.9614, 3.2168, 881220, 89, '2021-08-18 21:48:53.532000', 8826, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611497691, '415480074adf1aa6fe4b2efcf0dc33cb031defc86771f9c6918eead972471f0d', '2021-08-18 08:35:03.000000',
        17, 51.2874, 4.2756, 421496, 0, '2021-08-18 08:38:20.589000', 266644, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611504928, '9bf5807858cc313d7bd949844b33b97a4ed629908112e0d11f7dc0f8bbca8b7b', '2021-08-18 09:07:33.000000',
        33, 53.426845, 11.186995, 866299, 48, '2021-08-18 09:07:37.005000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611496488, '49fe023c2a83a94e0a9587b9ae33f9065073bf5a7d5439783a1d23cc2cffced0', '2021-08-18 08:33:22.000000', 4,
        47.8499, 1.8042, 572238, 57, '2021-08-18 08:33:23.748000', 348815, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607952294, '065e9f07dc12dbb048fc765c45ea808d93b52463fa681bccd610123cfe2a2b8f', '2021-07-28 11:20:04.000000', 5,
        50.453, 2.9789, 196094, 10, '2021-07-28 11:21:03.494000', 68649, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607960551, '8985c4db70a37aa1d6e64dfd56cfc88b3ea5cdea8612de8dd30528b44440f146', '2021-07-28 11:50:44.000000',
        35, 51.3834, 6.2285, 646567, 0, '2021-07-28 11:50:47.224000', 408119, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611484098, '100ce180dcf3d5796ac2abed527170b7b4c9a07877cb91ad8742162726b54227', '2021-08-18 07:21:08.000000', 7,
        50.9819, 3.2381, 38372, 52, '2021-08-18 07:43:22.804000', 14552, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611481881, '1378bdd4214bc63f4588149162b0c15b760904818cade4d3e95c4ffabfd8974c', '2021-08-18 07:34:23.000000',
        34, 51.54014, 4.2945, 457056, 87, '2021-08-18 07:34:25.412000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611632481, 'a1ad85139997d53184804439688a85f0e2ca05dcbc5fb9a09edec8f3ec091e6b', '2021-08-18 20:49:15.000000', 4,
        50.0887, 3.227, 250473, 5, '2021-08-18 20:49:54.091000', -1, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611538506, '933ca4540b0016310678e138f21cdf1c6a82472ee0047346a6b9224b2bb3ebee', '2021-08-18 11:34:07.000000',
        33, 51.0238, 5.2051, 285456, 86, '2021-08-18 11:34:09.466000', 171577, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611493428, '2680531799e303e3955b84954897286bec5fe2ba3e0f2896549e9b0074d7b25e', '2021-08-18 08:20:40.000000',
        25, 50.5159, 2.8439, 427829, 12, '2021-08-18 08:20:43.338000', 231156, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611479588, '2ff2faeba86b5b923e5e2a9702d785f7ce176b1cd69d733af14224ef4f782f01', '2021-08-18 07:25:00.000000', 0,
        47.864824, 8.784597, 749160, 0, '2021-08-18 07:25:26.887000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611698700, '8c8c25309d0278d1131f126524b8da5182facd50365cf215557744f2ba9de032', '2021-08-19 06:52:11.000000',
        20, 44.5061, 4.7713, 39329, 89, '2021-08-19 06:52:14.009000', 22900, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1608643503, '1e25000ee8ed6e694c94e5400da2649ab97e6d3092a0ed4ddae9364048107931', '2021-08-01 09:48:31.000000',
        16, 54.98716, 9.3776, 180371, 84, '2021-08-01 09:48:31.512000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607910548, 'ba01fd30a91c1c817165944ae5059e2851bd24617505499f0097cf1907a1e67d', '2021-07-28 08:47:55.000000',
        18, 48.7415, 2.3865, 564489, 0, '2021-07-28 08:50:02.824000', 344316, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611538243, '92931c515aa3cf80dd8a0fd7d9a339142afb92851c0dab36fb14ef7bf446ae5b', '2021-08-18 11:32:54.000000',
        33, 50.09765, 6.86732, 353633, 84, '2021-08-18 11:32:57.431000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611490068, '5d1365ed29b545dece43eff54b24c57d32a4c9e814a6b748f65efc142405b136', '2021-08-18 08:07:03.000000',
        30, 51.1945, 4.3962, 365864, 89, '2021-08-18 08:07:02.466000', 221300, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611635351, 'f1136eab4d12d03deaf7f3cc5d30717d3896738cdf5375b7345c02c43575ab71', '2021-08-18 21:39:03.000000', 6,
        51.7734, 4.9531, 280070, 1, '2021-08-18 21:40:04.823000', 149894, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611611091, '393dc9f1bd7ee63f36984f7ccb9f055528b1ad978432e1178beb631b9cfdc843', '2021-08-18 17:02:39.000000',
        18, 51.7986, 4.9309, 539280, 89, '2021-08-18 17:02:39.372000', 317085, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611544355, '49886bb2d99b7aa0374346decaded67d6333ce3d8332971f4f1a591dbbbc32d7', '2021-08-18 11:59:40.000000', 1,
        50.4747, 3.0042, 422641, 0, '2021-08-18 11:59:42.698000', 205706, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611621573, '8995699fdc56f8ba4c01d87200afd477458a7f7588c213531a1b7fde0ebf35e7', '2021-08-18 18:26:17.000000', 0,
        50.7949, 3.18066, -1, 0, '2021-08-18 18:27:21.002000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611512084, 'd336bdeddf8dac3505d230235f23275cc78c01f327827f127081f404f93ff737', '2021-08-18 09:37:14.000000', 0,
        48.9904, 2.6504, 691624, 0, '2021-08-18 09:37:15.516000', 269265, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611622080, 'de900dd366d1fd3f71370f2440a3ae62a56191625f9f4b7aeb33d4586b3c3bb8', '2021-08-18 18:32:42.000000',
        30, 49.0566, 8.2288, 106722, 88, '2021-08-18 18:32:41.909000', 54262, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611489896, '050ba61baa090f3ec1a787b758367c2ae87c025276f8dd34e8f6f327e7763d22', '2021-08-18 08:06:20.000000',
        18, 47.4133, 8.551, 914802, 46, '2021-08-18 08:06:20.697000', 496381, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611601730, '1273ecfc12e987eb81bf86344c66fb75c1707da013724034ff374ff9041e7abd', '2021-08-18 16:08:20.000000', 7,
        47.9038, 1.4684, 244290, 56, '2021-08-18 16:08:23.003000', 144077, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611521378, '31516556a1f17a50fb9e1527f73cc53eca787958d1947ef1eac71204e7cd22a1', '2021-08-18 10:17:56.000000',
        22, 50.8559, 2.7551, 822914, 0, '2021-08-18 10:17:55.321000', 555546, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611623337, 'afad8ffabece80a1e127fcf71292b185c8c0bb01b1c81f5b9a84c33be2eaed35', '2021-08-18 18:45:14.000000',
        34, 50.8694, 2.8943, 364, 0, '2021-08-18 18:45:14.984000', -1, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611620060, '1d2f251bc9c51ede1af0bd6325e18b9206d7b947c80810a4ea0792c1c7d1a2d0', '2021-08-18 18:11:57.000000', 0,
        51.204637, 7.247885, -1, 39, '2021-08-18 18:13:09.352000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611638163, '67d51505ada91af78afad5cd976327eb12e3b7eb7f379e971b643ac162a6d130', '2021-08-18 22:31:26.000000', 9,
        49.8865, 5.3742, 1327530, 87, '2021-08-18 22:31:25.469000', 713212, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611490175, '956cff4b0fc01cb70ba12db1e679f1ecffef55a936c565ef562c8ac70cf0857d', '2021-08-18 08:07:29.000000', 7,
        50.9466, 3.1002, 582133, 1, '2021-08-18 08:07:29.566000', 358885, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607964139, '50461fe666deb72f83cf84a411f60431039d8e5dded7628b2caa877673a8d9ee', '2021-07-28 12:04:16.000000',
        35, 50.3852, 3.5547, 506533, 35, '2021-07-28 12:04:17.027000', 243641, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611544357, '5d1365ed29b545dece43eff54b24c57d32a4c9e814a6b748f65efc142405b136', '2021-08-18 11:58:43.000000', 7,
        51.1197, 4.3701, 365955, 1, '2021-08-18 11:59:43.683000', 221347, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611511363, 'a05e672f92a07256f8bacfbc56e7bd75fb4830404cfd064db1a38f8cb97a8415', '2021-08-18 09:31:07.000000', 5,
        50.0891, 3.2272, 227668, 0, '2021-08-18 09:34:20.749000', 168580, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611634085, '7c8b04567ced592121221f20a0ff932be873d4d3e7837003b7cec70b400ecd43', '2021-08-18 21:18:55.000000',
        35, 48.7561, 2.3641, 669854, 7, '2021-08-18 21:18:58.375000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1608935408, '346897b60c4cd05f4850b375afb69e71d419da3fdb145bcd32531d5f4509c80c', '2021-08-03 04:47:12.000000',
        34, 51.28036, 4.23921, 99572, 0, '2021-08-03 04:48:23.964000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1608727236, '121868513d3bbe24e72ad866b8202d28591b1e94d026dc76adcfb8497f923cfa', '2021-08-02 06:52:53.000000',
        35, 50.8693, 2.8942, 364, 0, '2021-08-02 06:52:53.692000', -1, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611638065, '251ffbf1c3a0440f9132f3f7217e4308702e27137427891c34839ac30c742de9', '2021-08-18 22:29:32.000000',
        34, 52.833343, 8.442758, 156528, 0, '2021-08-18 22:29:36.834000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607958718, 'db4d58549778bb51f4716b8c2d2e0c9b369c2b3581a2d722747eb74bb3c3e0b3', '2021-07-28 11:44:05.000000', 1,
        49.88994, 2.84013, 290276, 90, '2021-07-28 11:44:05.764000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611635945, 'cb622613ea0ff1c2597c037624a717328bf723a8712e69085032f53f4faf41f9', '2021-08-18 21:49:52.000000',
        18, 48.45495, 4.157949, 9024, 91, '2021-08-18 21:49:54.781000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611633702, 'f1f8a5f50fe9558d864abc0022662687d61444d197481048f81f7383dadbe898', '2021-08-18 21:11:57.000000',
        16, 48.4243, 2.5403, 759099, 64, '2021-08-18 21:11:59.170000', 154046, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611503778, 'fdcb9d1e2c8ae61b559afe278cfe2a854f77a5d247014994918a708f8a760956', '2021-08-18 09:03:15.000000', 0,
        50.8406, 2.8763, 905, 0, '2021-08-18 09:03:17.104000', -1, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607911692, 'cd0c70fd67dd917cae6368297b1cf3bcff3f8c3cecf18edfede273978f558e76', '2021-07-28 08:48:28.000000',
        35, 51.5419, 4.294, 415543, 0, '2021-07-28 08:54:49.463000', 263152, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611484365, 'd8886421ba50c8c5503d0091d0dbb4c36df62ec46fd0fa37ce2f066efb7b48dd', '2021-08-18 07:44:19.000000',
        19, 53.252522, 10.087135, 454510, 84, '2021-08-18 07:44:22.208000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611639942, '19aabb226870d5b86b0d8b95ee550389f751f3c66ee3d835c06adc5b31ab326b', '2021-08-18 23:03:04.000000',
        15, 48.6437, 9.4285, 247625, 0, '2021-08-18 23:03:02.138000', 132833, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611539049, '233e7c4415c6a899f960a779eb6b284b596d636f259ce85707bc5e9e33fa771c', '2021-08-18 11:36:40.000000',
        11, 51.1985, 4.3811, 132759, 77, '2021-08-18 11:36:39.985000', 70668, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611509676, '123ed7eddb1e3b13a3e466ad42a705ba373e2097cb0292534c3a34edb23259d5', '2021-08-18 09:26:51.000000', 0,
        50.760227, 3.28929, -1, 0, '2021-08-18 09:27:24.862000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611676395, 'defb3d14f74bcf40082ae3d7edff927f264b7b5bc1b9111916b4a6de633ccf35', '2021-08-19 05:16:11.000000',
        28, 48.237, 4.4518, 315249, 30, '2021-08-19 05:16:12.791000', 172357, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607952764, 'a3e6ad45f6a68942e2b0a44bd382feca63d6197eeee5761b30c80ccdf0212c6f', '2021-07-28 11:22:44.000000', 5,
        47.9555, 5.4611, 332027, 26, '2021-07-28 11:22:46.470000', 195107, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607910895, 'ccf5526be6667e68db8781d4277573bad0cfe01b9947274ffa1b7654fa8e6b33', '2021-07-28 08:51:27.000000',
        12, 50.8698, 2.8947, 0, 0, '2021-07-28 08:51:27.988000', -1, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611528417, 'e0939a55640bf2b5503f50ad8d10e7619d4d683fa1719d4010f02fa726d90cf0', '2021-08-18 10:49:44.000000', 7,
        50.3344, 3.0876, 682154, 71, '2021-08-18 10:49:41.788000', 336056, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611602806, '297345fe2dfbbe21240599738a497f3b53dd1c78f5f2ccd1361b4f368002e75c', '2021-08-18 15:58:04.000000',
        15, 36.81212, 21.53507, 152010, 39, '2021-08-18 16:14:34.186000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611580066, '854ba69cec35f6f3a7f04eb1b9cc091504d6e0119d998583104f49f3ed55ee27', '2021-08-18 14:27:53.000000',
        30, 50.3675, 3.1258, 497359, 3, '2021-08-18 14:27:58.106000', 20642, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611621366, 'be34e23cd46f10b79b21624cf511ff6514804e350fe5ef1cd248bfc8fa2dc05c', '2021-08-18 18:23:47.000000', 0,
        45.21996, 9.07544, -1, 0, '2021-08-18 18:25:21.387000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611648698, 'b4468a33f0875d3c5db99b7c4ed87b4c34ae29f09a9b91a19292b380d2d4a690', '2021-08-19 01:33:48.000000',
        22, 50.970333, 3.607708, 380395, 0, '2021-08-19 01:33:53.234000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611493456, 'deaa5ddce8433b459bd2e58cea13013907ce0c36d97894925271fc4093bafb6b', '2021-08-18 08:19:53.000000',
        33, 53.280143, 11.096942, 114444, 64, '2021-08-18 08:20:55.110000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611500267, 'ddf29b30e22603c443028af98c751b8659bf14fdc2e0dbf13a839f2e24b7370a', '2021-08-18 08:49:12.000000', 9,
        51.193, 4.3153, 487908, 0, '2021-08-18 08:49:14.094000', 8830, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611602797, 'ac98104bb511d4eefb32df7bfd052ecc5047a55ff61c0d54571051eeeaa43be9', '2021-08-18 16:14:00.000000',
        22, 50.2695, 3.3246, 471371, 5, '2021-08-18 16:14:30.499000', 227625, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611521546, '0942ef5b9ed33db50b9a4ede949d29f9a67f7417f12b6332afed831246bc4f8d', '2021-08-18 10:16:34.000000',
        20, 51.2622, 4.2426, 529672, 5, '2021-08-18 10:18:36.644000', 85571, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611488613, 'ce2e62efea8abfcfeba28c0b7536d5837dd9663bd5af9212bcc24b43126191ea', '2021-08-18 08:00:28.000000',
        31, 50.71824, 3.19675, 254869, 0, '2021-08-18 08:00:35.512000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611501150, 'b465304b5f1c8351c8c99b1dc7db84aee35f45843090f908a30bb9b4350f45c1', '2021-08-18 08:52:41.000000',
        21, 50.4464, 2.9731, 66382, 0, '2021-08-18 08:52:44.125000', 40857, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611645135, '5618b3d6a3f4b958ad85ba4a292b868505dae591f6e955195acf2808e8fe2e66', '2021-08-19 00:36:00.000000',
        32, 50.567681, 7.142675, 543162, 87, '2021-08-19 00:37:06.825000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611633025, '82b9caf10f076b50f34ed64d3fdb8f14278d57ac0ffff083d6a85d990544e34e', '2021-08-18 20:58:00.000000', 0,
        51.773242, 4.952278, 741822, 0, '2021-08-18 20:59:14.960000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611537975, 'f36f135eac01c9154f42c24e4004f038d7e5ae922a7a63275a13309f447ea205', '2021-08-18 11:31:25.000000',
        28, 48.92308, 2.34438, 102097, 71, '2021-08-18 11:31:32.088000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611482625, '955e1c577b527c403a57d8b3130b9c48655253f087085512548dc525ce2a5d0b', '2021-08-18 07:37:19.000000',
        26, 50.4086, 4.4055, 372815, 0, '2021-08-18 07:37:18.581000', 230299, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611633902, '77c16f14ba90e88b6f566acb47fe3e07a632932e6fc2ca74178b11d996a3f239', '2021-08-18 21:15:58.000000',
        35, 48.7558, 2.3657, 582502, 5, '2021-08-18 21:15:58.290000', 359114, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1608727976, '210d4f3101182622eadbdb4f866cfe2c342282b7600f53554bc392500d79127e', '2021-08-02 06:55:39.000000',
        10, 48.2364, 4.4511, 685656, 0, '2021-08-02 06:55:41.652000', 265744, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611621303, 'e1b78e0db2fc81b652052ebcf9ea90fbafd3f86735203ee1d89dc13138304344', '2021-08-18 18:24:43.000000',
        32, 51.5562, 5.361, 458381, 0, '2021-08-18 18:24:48.861000', 243963, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607432606, '20a82ee97269b011495eab6f70d17d95f0dc138335b03ed8862015d51692e430', '2021-07-26 10:57:19.000000',
        34, 50.3531, 3.0311, 364058, 0, '2021-07-26 10:57:20.681000', 198361, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607954879, 'ba01fd30a91c1c817165944ae5059e2851bd24617505499f0097cf1907a1e67d', '2021-07-28 11:30:17.000000', 1,
        49.7123, 2.7732, 564612, 90, '2021-07-28 11:30:17.186000', 344382, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611508096, 'e465ac5a8c58a1463696d7f3d6ff970b314a99d5932c964f88f79da6d066b0b0', '2021-08-18 09:20:25.000000', 3,
        51.39686, 4.6861, 723643, 30, '2021-08-18 09:20:24.368000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611648560, 'bec14ce53afdd1f632e663514cc7389ec715f22ece65b3d4001f837fbcfd00ce', '2021-08-19 01:31:48.000000', 9,
        51.7729, 4.9526, 1299588, 1, '2021-08-19 01:31:47.668000', 716417, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611591319, '6cec01c5bd343397a22849e63ba84ca5a14ae1ff8e6431176053c29bdc4c50c9', '2021-08-18 15:17:21.000000',
        19, 46.04161, 5.32344, 307935, 71, '2021-08-18 15:17:23.487000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611582835, '28987f830055429917e21b11c355c501fb846f7e2d9af6f239e18e7ea053d637', '2021-08-18 14:39:44.000000',
        15, 48.8444, 2.3643, 836279, 0, '2021-08-18 14:39:43.999000', 537784, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611622084, '740c2122c7f2a590241e4e0cdce5d537f7a8f904ad0ab653ab088e8ce4e4d3de', '2021-08-18 18:32:45.000000',
        29, 51.2409, 4.2409, 413605, 64, '2021-08-18 18:32:44.771000', 232864, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611513812, '2c9b977bf40e502c556347b020f1cc9e4d12bd2940cf85b7a0803c72742a91f0', '2021-08-18 09:44:56.000000',
        23, 51.3886, 5.3626, 1044409, 84, '2021-08-18 09:44:58.575000', 736675, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611634332, 'bec87ebc3fd877c03173624514c2043337ebbd7779e7ede3644cdda36af072bb', '2021-08-18 21:23:20.000000',
        255, 50.8717, 2.8856, 68710, 0, '2021-08-18 21:23:22.779000', 37530, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611703787, '0a162ab2f63c3ef73110d07c00de34568474d1a7b4c4446dd316aec5b4bc80c4', '2021-08-19 07:13:15.000000',
        28, 48.2368, 4.4487, 836481, 0, '2021-08-19 07:13:15.193000', 537889, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607910866, 'df8ecb8bbf355ce7a6fc49300b30082cea7dedd0c77a55aa3a232c9bfc60e983', '2021-07-28 08:51:17.000000',
        23, 51.4587, 7.0502, 702137, 43, '2021-07-28 08:51:20.388000', 439135, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611653439, 'd2c6e92942fe5c7f8b190eba59faa0e54453c94202e7580029f7898ebca9a616', '2021-08-19 02:35:28.000000',
        22, 50.8559, 2.7548, 822914, 0, '2021-08-19 02:35:27.930000', 555547, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611523669, 'f837c779a69dc928669ed6b78c204694d2a6b138e5127de381fd0c022434ea20', '2021-08-18 10:28:09.000000',
        34, 50.8691, 2.8945, 28442, 0, '2021-08-18 10:28:13.117000', 1409, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611601864, 'b7c95010e99a40373cec1e81ff528155c42e163d21a82068c87bac6b9496adc6', '2021-08-18 16:09:09.000000',
        17, 50.3691, 3.1203, 199940, 34, '2021-08-18 16:09:12.579000', 70885, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611639953, 'c7014d6c2102e871c5cb25c05483417b883c0996d4ac4afb6a012ec2deaa3ece', '2021-08-18 23:03:10.000000',
        13, 47.5878, 2.8545, 818216, 79, '2021-08-18 23:03:11.004000', 265951, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611503812, '4db9de89cb3de9a968b1437891d0255bfa824bc03bb9cc39d26a63c12f0c451c', '2021-08-18 09:03:27.000000', 7,
        51.0448, 5.2034, 25717, 66, '2021-08-18 09:03:27.530000', 16406, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611493627, 'dadbad51d5f743fbc3c06acfe4c60b8aaecd605be5fd3578e3262868e8ddd21f', '2021-08-18 08:21:00.000000', 0,
        47.62513, 7.608312, 718711, 0, '2021-08-18 08:21:32.441000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611500955, '6ce37a5b9eeb12ee3388acd356f2e309c81537b8099c9c74326eff77c350876f', '2021-08-18 08:52:00.000000', 6,
        50.4745, 2.9846, 780019, 0, '2021-08-18 08:52:04.539000', 398637, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611583613, '1eeb3aab80e3070fdde61c28f6b788364ad3939c462ad00c1336d7241b89db39', '2021-08-18 14:40:59.000000',
        34, 50.8886, 2.8739, 390025, 0, '2021-08-18 14:42:58.712000', 213598, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611512636, '3387d1de3754b5e181226ba1da3d3b609db94d5a6324610a53317a39a8f3a0e0', '2021-08-18 09:39:49.000000', 6,
        51.0585, 3.8555, 279031, 79, '2021-08-18 09:39:47.690000', 139166, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611591299, '4ef185aac17dba25889bc76962752f5455103d801c94cef8ac3a58c6e27ab459', '2021-08-18 15:17:13.000000', 4,
        48.2369, 4.4516, 340766, 0, '2021-08-18 15:17:16.984000', 194166, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611512584, '67a04f426e42bad84c6479d638a2f3f3ca0c05e71ecbdd4cf8c6ec1ac729db3a', '2021-08-18 09:39:30.000000', 5,
        39.44863, -0.74171, 509824, 91, '2021-08-18 09:39:33.766000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611490281, '956cff4b0fc01cb70ba12db1e679f1ecffef55a936c565ef562c8ac70cf0857d', '2021-08-18 08:04:29.000000', 7,
        50.9468, 3.1001, 582133, 0, '2021-08-18 08:07:59.872000', 358885, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611653252, '2b068f95c87633bda77d9b5cab626ee036fa2fb7191d83562cc6e25911c041ba', '2021-08-19 02:32:04.000000',
        33, 51.7739, 4.9518, 107244, 11, '2021-08-19 02:33:06.633000', 54495, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611496048, 'db6651e098793abaadbf4c0b52bff07d0f138a849d117449514d3d8227584eb7', '2021-08-18 08:31:40.000000', 6,
        51.9312, 4.4391, 81717, 73, '2021-08-18 08:31:41.643000', 46081, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611544307, '9dde6da4ebdba15e7dba39933c68f5776c3b316f52d21386618c8f2d5c5eca2f', '2021-08-18 11:59:29.000000',
        16, 41.13025, 14.8053, 1114740, 84, '2021-08-18 11:59:32.196000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1608727998, 'ed16b03a24c24c3a03a4c2d154b20f179b95e604fdf23ffe442d305dfcae7e39', '2021-08-02 06:55:45.000000',
        10, 49.05, 1.9565, 809450, 6, '2021-08-02 06:55:46.355000', 403840, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611503809, '217b05c736fb21c55a2f172334eefb27849cf422b7e219dc2c99f1e828de69ee', '2021-08-18 09:03:25.000000',
        25, 51.61231, 7.75824, 320239, 70, '2021-08-18 09:03:26.945000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607958025, '0db6f2cc9dabd965aef8f4e1c034f75bcf3eb415bf706dff37ea076a9104b07f', '2021-07-28 11:40:13.000000', 9,
        47.8975, 0.9664, 366524, 84, '2021-07-28 11:41:40.430000', 84841, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607910382, '71f1c8230a266ada2b27834d090482668cb664f91d1312b0bf0a2a5768102c77', '2021-07-28 08:49:29.000000',
        34, 50.4504, 3.0118, 100905, 0, '2021-07-28 08:49:32.452000', -1, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611635446, 'f1f8a5f50fe9558d864abc0022662687d61444d197481048f81f7383dadbe898', '2021-08-18 21:40:27.000000',
        21, 48.1288, 2.7142, 759142, 90, '2021-08-18 21:41:23.239000', 154067, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611532472, '5b947bbfbbe4eda4991a32b0a25a4c075ceff0c5775e3c75688d2833ba5efc45', '2021-08-18 11:07:17.000000',
        10, 48.2368, 4.4504, 145360, 31, '2021-08-18 11:07:19.042000', 37665, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611523353, 'd336bdeddf8dac3505d230235f23275cc78c01f327827f127081f404f93ff737', '2021-08-18 10:26:44.000000', 9,
        48.958, 2.8442, 691651, 83, '2021-08-18 10:26:45.707000', 269279, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611646120, '1ec09d033286ef44bbc3c78d98a447a84ee82e66491f8a868fe0cfc3d29097a1', '2021-08-19 00:53:51.000000', 6,
        48.9352, 2.2756, 786704, 5, '2021-08-19 00:54:05.770000', 414877, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611580106, 'ddd96d0ee0c0260f9349f078373d15b29787f2f2e85d8cded7a197ed77ac5d51', '2021-08-18 14:28:10.000000', 0,
        45.5835, 5.9499, 675425, 0, '2021-08-18 14:28:09.311000', 318506, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611638886, '978eb3b740f9521be2a374cbd68a33d3019676e8f4322e2a94b0882b2f49f9e6', '2021-08-18 22:43:36.000000', 6,
        50.3698, 3.1234, 449777, 0, '2021-08-18 22:43:41.942000', 203522, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611698787, '520ef68851d8eac95acf38da057fc783934bc703c4714b38e8ec4c285a3770ff', '2021-08-19 06:52:32.000000',
        29, 50.8731, 6.0356, 502947, 0, '2021-08-19 06:52:34.404000', 290661, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611601839, '05f87bbe565a67b44c3daec58aa58d689c2ba9a49a589d88bf8806a289a7d814', '2021-08-18 16:09:06.000000', 0,
        48.863, 0.9541, 236623, 0, '2021-08-18 16:09:06.843000', 183744, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611611184, '492950ae71662fe89e1818da06aa58e3f2636c39e1aa67fccea91741f5c9a9a2', '2021-08-18 17:03:16.000000',
        25, 50.8559, 2.7549, 853077, 0, '2021-08-18 17:03:13.487000', 587172, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611490215, '92568237a4f298d8fed00c56f12ccc9a83f6bc1d5bbb7c5b58ec6255737c5d88', '2021-08-18 08:07:36.000000',
        23, 50.814, 3.2861, 687688, 87, '2021-08-18 08:07:38.616000', 395139, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611676935, 'e177f5ae2824a45b1a25e0648f31de7581975dbb390d09b699cb1e5b3bdb48ab', '2021-08-19 05:18:52.000000',
        30, 53.136212, 10.490605, 173817, 49, '2021-08-19 05:18:54.312000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607951158, '1ad966aaa0671def1da06668960b053f7b21527cb6f35b0518ac98c4567a2189', '2021-07-28 11:16:20.000000', 4,
        50.9134, 3.4845, 408653, 28, '2021-07-28 11:16:20.219000', 204998, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611632584, '932400cc64633a500f3259e13b0214294d7ee2f7505182a484136ceea5be1eed', '2021-08-18 20:51:54.000000',
        13, 50.3714, 3.1107, 594606, 0, '2021-08-18 20:51:56.119000', 314296, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611502264, '1ec49df62a5e78f4ec6c9d69a1d061c41d7ff995d54293dcba1e576e656a484d', '2021-08-18 08:57:10.000000',
        11, 51.02031, 3.27938, 506688, 29, '2021-08-18 08:57:08.655000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611648776, 'c39f263f203d4097f668dba6a5c9077ea74e8e8bacb1e5165f2a5a76b2650889', '2021-08-19 01:35:03.000000',
        28, 47.4697, 4.2284, 881473, 89, '2021-08-19 01:35:05.251000', 8949, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611583166, '8a7e85f6f6ef4661b47098c0e88625e8cc5243f791719650d02f4bf6e71c9669', '2021-08-18 14:41:03.000000',
        19, 48.7321, 4.2041, 149296, 20, '2021-08-18 14:41:03.068000', 85735, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611549726, 'b663048143e095be052d438e09b92cbef1a0ac16dc5bd2d9ebbb84a68f067994', '2021-08-18 12:14:18.000000',
        31, 50.0897, 3.2311, 233821, 0, '2021-08-18 12:21:43.173000', 128326, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1608728287, '8d94e3cfa888c2d28b9399cbcde645c6889034db51123d775228e27aa547e6a2', '2021-08-02 06:56:53.000000',
        20, 46.5224, 4.916, 648098, 90, '2021-08-02 06:56:55.708000', 396520, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611479064, 'c50868242dab0e4788953277d74e536d8122ba2d5ded185d24d063d49b222a6f', '2021-08-18 07:23:08.000000',
        35, 53.316757, 10.156463, 313396, 0, '2021-08-18 07:23:09.457000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611635865, 'a1ad85139997d53184804439688a85f0e2ca05dcbc5fb9a09edec8f3ec091e6b', '2021-08-18 21:47:49.000000',
        32, 50.0781, 3.2159, 250512, 28, '2021-08-18 21:48:28.928000', -1, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611523731, '19aabb226870d5b86b0d8b95ee550389f751f3c66ee3d835c06adc5b31ab326b', '2021-08-18 10:28:34.000000', 9,
        49.2749, 8.7437, 247442, 88, '2021-08-18 10:28:32.091000', 132721, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611482098, '34f9f0b51ba7955dfa6608891f9852adde5310a87c00c06d8eb4fcb037d15d05', '2021-08-18 07:35:14.000000',
        26, 50.4346, 2.8722, 728300, 80, '2021-08-18 07:35:17.940000', 159752, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611493468, '5ff4bc9930f789b9b2c407466607b4ffd046aee2880e78609ae3b30d3b432259', '2021-08-18 08:20:54.000000',
        11, 48.3285, 4.0516, 343186, 81, '2021-08-18 08:20:57.898000', 185128, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611623354, '734a8eb54f88ed21909b48065b1c67de9f2cbcb466398474f5c5611ef7aed52e', '2021-08-18 18:45:25.000000',
        16, 49.2796, 2.6957, 627629, 89, '2021-08-18 18:45:24.856000', 369754, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611537820, 'f837c779a69dc928669ed6b78c204694d2a6b138e5127de381fd0c022434ea20', '2021-08-18 11:30:45.000000',
        22, 50.8512, 2.8762, 28471, 20, '2021-08-18 11:30:47.533000', 1416, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611498129, '6c0bb337a2db89ec262d82f611946c678ac1d21d4154ae3d3d13dbbe6ef37c5f', '2021-08-18 08:40:24.000000',
        21, 51.7333, 8.7066, 94015, 23, '2021-08-18 08:40:25.958000', 48811, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611601881, '09f1a280dad98525233730e9cb23840c50d7a7d7a3b5ed2879dd221fe7bf819f', '2021-08-18 16:09:17.000000',
        15, 49.9344, 1.6424, 297730, 97, '2021-08-18 16:09:16.892000', 178814, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611602351, '853d4ade678024b335f0acd41189622a8ba8383fc074f5a1b1dbadb0d2877d48', '2021-08-18 16:12:02.000000',
        34, 47.5595, 2.8829, 143049, 89, '2021-08-18 16:12:04.913000', 78397, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611621347, 'ad07f549f4c6b4b8d563c812490760abcfd724b3e8428c49f003a230bb148e59', '2021-08-18 18:25:12.000000',
        19, 50.8719, 4.6548, 506437, 88, '2021-08-18 18:25:13.413000', 165168, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611648007, '333d06fd8843c10f9838f224d112f127fb55d4e6a9e68ee9dab08cd641a905bd', '2021-08-19 01:22:05.000000',
        10, 48.9824, 2.6442, 627831, 0, '2021-08-19 01:23:22.672000', 369852, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611653407, 'c21492ae7e30fdfbf2fea95222f58141dd6937fcd1ced89d25fcd8039296324e', '2021-08-19 02:35:05.000000',
        25, 50.8693, 2.8943, 364, 0, '2021-08-19 02:35:06.554000', -1, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611500252, '4deed64ec3a900e72cbbefca46b5d748dc24ae0c40ed8ee64a181be26cc6513e', '2021-08-18 08:49:09.000000',
        33, 52.89695, 10.65078, 173603, 64, '2021-08-18 08:49:11.460000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611539199, 'ce490a4f5f817ed08c291a04d7b418aa07136ec7c1f90f34122dd906e66cf05c', '2021-08-18 11:37:09.000000',
        31, 48.53072, 6.22149, 133320, 78, '2021-08-18 11:37:13.006000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611484316, 'a24e3d3a5c65a88609765cbd78225049a4638c5f2a14d5d433e14df5d2ac2a02', '2021-08-18 07:44:09.000000', 6,
        53.4334, 9.9076, 17612, 48, '2021-08-18 07:44:11.715000', 8925, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607911691, '1cf5bd69bcdf43e584a52fbe464e20f7123f5eed3745107be36ff5f90035b4d2', '2021-07-28 08:54:45.000000',
        11, 45.68067, 4.85485, 137685, 6, '2021-07-28 08:54:49.041000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611499069, '927acaf9b6f4b2033a22a62f3b23e5ff0fdfa2f64a27086712fd05c264664f9c', '2021-08-18 08:44:14.000000', 9,
        44.9144, 8.22275, 431939, 87, '2021-08-18 08:44:16.840000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607959183, 'df43a8e9c60ae0f6fa65860af1ddf6f9cde1193c6aa893b9f876a69dd578b790', '2021-07-28 11:44:37.000000',
        34, 49.446, 3.9283, 830916, 20, '2021-07-28 11:45:41.283000', 414894, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607951517, 'daca3de49700c66bcf49147698a031bf205f1f5a9d057ac06cb871fed79fff6f', '2021-07-28 11:17:56.000000',
        28, 46.0334, 4.0987, 1035887, 18, '2021-07-28 11:17:55.338000', 638082, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607958008, 'd3b29c7c71f29f629d6003ddd6a39f8cbc8cf3510418361d1a8b19df132c2b3e', '2021-07-28 11:41:35.000000',
        25, 48.3284, 4.0779, 704355, 91, '2021-07-28 11:41:36.946000', 400369, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1608649138, 'b4e59348902b2ca6cec72293ab2569ed7f486e284d2b7bb77c28a7d4d2698f19', '2021-08-01 13:27:35.000000', 5,
        50.871505, 2.885318, 37626, 0, '2021-08-01 13:27:42.180000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611535538, '99817245946395ea0c9f6fa0c0a128a4fcd5220888f0836809852387926c29e1', '2021-08-18 11:20:21.000000',
        29, 47.05771, 4.82517, 230492, 84, '2021-08-18 11:20:23.227000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607911669, 'f42c60440c22300bc1a17ff98a4edd6a4603eeda61e2fa2a2664f87d9d52e024', '2021-07-28 08:54:41.000000',
        18, 47.9278, 1.9396, 330579, 67, '2021-07-28 08:54:41.869000', 60851, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607910927, '2dcda21853e21895f1099689e95d1f6682812dad25e2cb10988e95cff21aaf48', '2021-07-28 08:48:59.000000',
        18, 48.7415, 2.3865, 564489, 0, '2021-07-28 08:51:35.685000', 344316, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611536925, '050ba61baa090f3ec1a787b758367c2ae87c025276f8dd34e8f6f327e7763d22', '2021-08-18 11:25:45.000000',
        21, 47.0808, 8.3061, 914874, 53, '2021-08-18 11:26:41.426000', 496440, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611484366, '6e529795d1ed652f211a3db9ddca3d5f0ab48dfaa9a27757e2681814783fdd1e', '2021-08-18 07:44:21.000000', 3,
        51.0236, 3.7355, 270598, 88, '2021-08-18 07:44:22.545000', 149379, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611658723, '9dd15d84462858241dfe1fc318b7d42fd41d4fd164e0c541bf51b76871631fa2', '2021-08-19 03:28:59.000000', 7,
        51.84976, 8.31608, 328470, 7, '2021-08-19 03:29:10.623000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611639924, '675b59cb4d7dddf3844153a01bd24ed29bdac06864bbd14f99fafbb184babf61', '2021-08-18 23:02:33.000000', 7,
        48.5127, 1.7703, 623852, 89, '2021-08-18 23:02:34.289000', -1, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611648872, '490131889eaf8855235998d7115ee83765d442a7825e45ef30d46ebfa98c7617', '2021-08-19 01:35:53.000000',
        26, 50.3679, 3.125, 581014, 0, '2021-08-19 01:36:10.658000', 196608, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611657740, '1ec09d033286ef44bbc3c78d98a447a84ee82e66491f8a868fe0cfc3d29097a1', '2021-08-19 03:19:40.000000', 2,
        50.5279, 3.0443, 786912, 90, '2021-08-19 03:19:39.559000', 414969, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611501352, '9b19ae86a600d7b83a72c862fd0302c0afc385f6de09deed194deedd8f3e265a', '2021-08-18 08:53:30.000000',
        16, 49.083, 6.106, 921346, 90, '2021-08-18 08:53:31.425000', 172754, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611623945, '699203d3b1e245ebb3ec363a441e0bcbdd28c74d7bf8134782bc92d68ead45db', '2021-08-18 18:52:10.000000',
        24, 51.2018, 4.3194, 144384, 37, '2021-08-18 18:52:10.872000', 36230, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611633718, 'a5a7851d0f93ed76b6c85f0680ffc5f58bcd7319ac7d20f06329a0bf33850095', '2021-08-18 21:12:20.000000',
        32, 50.835, 2.5762, 694234, 90, '2021-08-18 21:12:18.361000', 367920, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611601922, 'd93557919f5c21389142bba2f0e8bb7009aadcabefbde31a6989c324b9418532', '2021-08-18 16:09:25.000000',
        28, 47.9942, 2.6239, 788503, 0, '2021-08-18 16:09:27.417000', 516326, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611520859, 'eeafd1a9c84f5121cf048addf1c68632b0044ddce5c45365d38ec416a5eb9d93', '2021-08-18 10:15:38.000000',
        16, 51.2213, 4.4505, 101935, 21, '2021-08-18 10:15:41.118000', 53804, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611621332, '1273ecfc12e987eb81bf86344c66fb75c1707da013724034ff374ff9041e7abd', '2021-08-18 18:25:00.000000', 5,
        47.9778, 2.6268, 244388, 79, '2021-08-18 18:25:03.159000', 144120, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607954695, '61d0893fc636d553c0d9a4a57b4171e0799c50a85e6a0df8dbd3c60905821b51', '2021-07-28 11:29:30.000000',
        19, 43.7695, 4.3256, 218238, 23, '2021-07-28 11:29:32.874000', 115437, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611622083, '5d1365ed29b545dece43eff54b24c57d32a4c9e814a6b748f65efc142405b136', '2021-08-18 18:32:44.000000',
        14, 50.8153, 6.028, 366163, 89, '2021-08-18 18:32:43.460000', 221446, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611658677, '9593b9395c7890b91087b013fbbd0be4e3637636686d7fb8a3185c92397ddd66', '2021-08-19 03:28:49.000000', 6,
        51.14338, 4.18426, 505495, 88, '2021-08-19 03:28:50.758000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607954845, '670e7d83c5bd5b92769fce8a222f3853621a4979dcd95163ebb606f73315a98a', '2021-07-28 11:30:11.000000',
        26, 50.971, 7.5542, 365468, 86, '2021-07-28 11:30:11.096000', 225887, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611676308, 'b5738bf92e601d0af7d2bf77ec8563aae72b3d416d250013194d5bb8bf92605e', '2021-08-19 05:15:44.000000', 4,
        49.9312, 5.6203, 501459, 84, '2021-08-19 05:15:43.661000', 308606, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611622010, '955e1c577b527c403a57d8b3130b9c48655253f087085512548dc525ce2a5d0b', '2021-08-18 18:31:56.000000',
        14, 50.8655, 5.966, 373071, 80, '2021-08-18 18:31:55.507000', 230442, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611679944, '6c65530bf53f387d2f8c1029d2cb07d3cb54599e45b31bbb4ded2bc90415dca8', '2021-08-19 05:31:36.000000', 0,
        47.7522, 5.3068, 227342, 77, '2021-08-19 05:31:36.083000', 124812, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607911058, '7ad2ef998f06da3b8d31f46d0a5f000f3339e9923d835f1b3dc51d46eaef915a', '2021-07-28 08:51:10.000000',
        24, 48.2253, 4.753, 646214, 60, '2021-07-28 08:52:11.356000', 395453, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611638883, '6b6f076451ab978dfd0690e973ec6801d11eb8b443ba7c1220ce9a76830ca9e0', '2021-08-18 22:43:39.000000',
        10, 48.0894, 4.8353, 706161, 89, '2021-08-18 22:43:38.639000', 365777, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611548920, 'cc4d6339beaa41c27a4c58b2aa9c7eecbc3f284fce6f4a072556b1ff50f53b94', '2021-08-18 12:18:35.000000',
        25, 53.5122, 9.9666, 474140, 0, '2021-08-18 12:18:36.727000', 252525, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611490100, '6ce37a5b9eeb12ee3388acd356f2e309c81537b8099c9c74326eff77c350876f', '2021-08-18 08:04:12.000000',
        11, 50.5655, 3.0353, 780003, 0, '2021-08-18 08:07:11.076000', 398625, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611583133, '42b20e9116f99dd9ceeb4aae435930d67adecc27d974446e62888ec23e4f4efb', '2021-08-18 14:39:40.000000',
        17, 45.14963, 9.9415, 1202, 0, '2021-08-18 14:40:53.408000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611539630, '02b25ab5bd9edba84a05a2b49e940e1d331846482ef4300f3168503b183ae617', '2021-08-18 11:38:06.000000',
        16, 50.7183, 3.2835, 21850, 0, '2021-08-18 11:39:17.330000', 1361, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611622074, '49fe023c2a83a94e0a9587b9ae33f9065073bf5a7d5439783a1d23cc2cffced0', '2021-08-18 18:32:36.000000',
        21, 50.3373, 2.9252, 572731, 90, '2021-08-18 18:32:37.298000', 349099, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611676332, '750e4724f5137c164254cf562b414bc8fd8a37404368e77373fc179571311fe1', '2021-08-19 05:14:07.000000',
        16, 48.4918, 2.6633, 839649, 78, '2021-08-19 05:15:52.471000', 419317, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611490210, 'bcbfe81f9ff07bdb9db33b07ac3e3507cd376e456e1d9ddcedd50ca3c76ceb7f', '2021-08-18 08:07:36.000000',
        25, 51.13052, 4.10325, 653257, 88, '2021-08-18 08:07:37.035000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611539047, 'fdbcd7dd6f616c47b54965d2ced5feaebc49b458fe8afd09d8cbf65608657dc2', '2021-08-18 11:36:28.000000', 6,
        50.3956, 3.0272, 524976, 0, '2021-08-18 11:36:39.618000', 265120, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611632513, 'ddd96d0ee0c0260f9349f078373d15b29787f2f2e85d8cded7a197ed77ac5d51', '2021-08-18 20:50:30.000000', 2,
        46.4783, 4.8882, 675664, 89, '2021-08-18 20:50:28.759000', 318616, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611611566, '95e1ee4b9b3af35177fc2929e39ae17cc806626a54d3ff392f31e25d85802999', '2021-08-18 17:05:37.000000', 3,
        50.736, 3.1295, 436460, 84, '2021-08-18 17:05:42.244000', 198213, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611579650, '99d94b914ffcb8c937c56fcf762be2a517fa014df1b63a25c726f87f376ad17b', '2021-08-18 14:26:06.000000',
        29, 50.9235, 2.9082, 253792, 0, '2021-08-18 14:26:06.933000', 184349, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611538051, 'f837c779a69dc928669ed6b78c204694d2a6b138e5127de381fd0c022434ea20', '2021-08-18 11:31:52.000000',
        26, 50.8489, 2.867, 28471, 45, '2021-08-18 11:31:54.671000', 1416, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611601071, 'b624189e6cd93c03fcd5e856c6c5bc41899a7b708f2b721e71d2ee4a7b9f073c', '2021-08-18 16:04:22.000000', 0,
        51.131313, 3.702846, -1, 1, '2021-08-18 16:04:38.339000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611497684, '4ccd3a9374b020734802c10c0fc4c956a0f40682529a239c6e3455cc45631513', '2021-08-18 08:38:18.000000',
        29, 48.1375, 4.3169, 250530, 60, '2021-08-18 08:38:18.552000', 145709, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611544324, '740c2122c7f2a590241e4e0cdce5d537f7a8f904ad0ab653ab088e8ce4e4d3de', '2021-08-18 11:58:36.000000',
        30, 51.1367, 4.3489, 413487, 0, '2021-08-18 11:59:37.415000', 232786, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611653460, '19d3f46b98827ae8a82bb1b932c523f3e514e0963b8939dd50e94e10181d829d', '2021-08-19 02:35:43.000000', 4,
        48.49007, 8.82758, 123141, 80, '2021-08-19 02:35:44.761000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611476791, 'c7d8e9f4c2c8312b3b3e3b4cf5175a3ad6cb961ad13b884938f1d8200c18b1e2', '2021-08-18 07:11:04.000000',
        18, 50.8634, 3.2626, 609542, 0, '2021-08-18 07:13:07.137000', 47402, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611544207, 'f0ddaf068fcb477673a9ecfa5f271d8789f9b7c16b855b2534b73df413247386', '2021-08-18 11:59:10.000000',
        34, 51.055, 2.6681, 253628, 4, '2021-08-18 11:59:10.239000', 192055, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611513224, 'e07e98d08683f3c721547912e0af4899875487afeefcf942fadbee6f8ad1e9b9', '2021-08-18 09:42:18.000000', 8,
        50.4502, 3.0148, 193309, 0, '2021-08-18 09:42:21.069000', 99965, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611549737, '0a2fc9c0d11d598fd849ee989a715a8682f08324f06c5a19cbbeef2bb4d5dd3c', '2021-08-18 12:19:45.000000', 3,
        51.2594, 4.2465, 915563, 34, '2021-08-18 12:21:45.532000', 79683, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611484536, 'b03e8b30e475cbe6dd8024b70447224012136057a3ff2e5b710361aa5d9493b8', '2021-08-18 07:45:02.000000', 4,
        48.3301, 4.1149, 251797, 0, '2021-08-18 07:45:06.623000', 228150, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607911329, 'd8d7d8e8d88b217055be015c16347320f3e20b61cd1de8a542e4656287f2115d', '2021-07-28 08:52:44.000000',
        17, 47.8576, 5.3508, 242907, 0, '2021-07-28 08:53:15.795000', 141004, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611633028, '492950ae71662fe89e1818da06aa58e3f2636c39e1aa67fccea91741f5c9a9a2', '2021-08-18 20:59:27.000000',
        25, 50.8559, 2.7548, 853077, 0, '2021-08-18 20:59:23.887000', 587172, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611549627, '970dd64e525ceb1219b82249d358fa6abac77b9ddd648619099bb7d964907a27', '2021-08-18 11:41:25.000000', 0,
        50.909752, 3.478677, -1, 0, '2021-08-18 12:21:21.004000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611567204, '9bee148e3fab10d0777a1bc7fff140ba677da3889a09224b5f8503266bb6d0ac', '2021-08-18 13:32:45.000000', 8,
        45.81619, 4.9726, 353964, 83, '2021-08-18 13:32:46.575000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611637938, 'afad8ffabece80a1e127fcf71292b185c8c0bb01b1c81f5b9a84c33be2eaed35', '2021-08-18 20:25:59.000000',
        255, 50.8693, 2.8943, 364, 0, '2021-08-18 22:26:43.090000', -1, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611480084, '099a440b6e18a065dea9d6f9609ffc0e6f5d904e693f0bf0cc1f8371a9f5c16d', '2021-08-18 07:27:21.000000',
        23, 50.806388, 3.266478, 344997, 0, '2021-08-18 07:27:23.590000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611480497, 'c6be029fa2eae9e388bfdb2695964fc4f0400f804c911df85eae84441bb05c42', '2021-08-18 07:28:56.000000',
        33, 51.09214, 3.20766, 19236, 0, '2021-08-18 07:29:08.361000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611476786, '23d30f0a4b5931e439ad09814dee46147aa957fdb5ec17db965b183e35a21852', '2021-08-18 07:11:53.000000',
        12, 50.44521, 2.97526, 6303, 0, '2021-08-18 07:13:06.150000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611548983, '5cff1dbb9d49b472e30dfad46de637e777e8cc50a21da8e6b2bc60129b4d38fe', '2021-08-18 12:18:55.000000', 7,
        50.8558, 2.7551, 645, 0, '2021-08-18 12:18:53.901000', 415, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611496587, '7e8b79241112a9a50aab9d0c095b29a19dc2266e467676c217326bf10c2792f7', '2021-08-18 08:32:40.000000',
        14, 50.44614, 2.97378, 29023, 0, '2021-08-18 08:33:52.770000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611622008, 'cbb020eaf9b47937e770c307448de3c14c21752d230a998d72a7a2b105d86aab', '2021-08-18 18:31:53.000000', 8,
        47.6662, 1.3868, 572107, 0, '2021-08-18 18:31:55.148000', 382326, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611600710, 'ceb5874e488e2f3b44339df4660df51ec843f6c462b1ff9ec319847f0370b1af', '2021-08-18 16:02:56.000000',
        18, 50.3314, 3.4369, 435107, 0, '2021-08-18 16:02:55.113000', 233045, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611548967, 'b633f9f4b423c2b7583a71feb9e285b63849016caf39ab3a547a63be09f116c5', '2021-08-18 12:18:48.000000',
        22, 49.2122, 2.6188, 881001, 81, '2021-08-18 12:18:50.636000', 8721, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611499386, 'dae8c0db6ea0f60f8fcd6e9ea65a6e591aba7b78624f5fed9d51c2b6d479e0f4', '2021-08-18 08:45:30.000000', 2,
        39.91464, -2.24677, 144051, 7, '2021-08-18 08:45:38.036000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611634220, '62934ee0dabda2d366e14dbc62035e72eb7cee9173381564e6db38ef85e8ce57', '2021-08-18 21:21:18.000000',
        34, 45.8045, 4.9257, 487648, 72, '2021-08-18 21:21:19.943000', 243106, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611499176, '9a6b07fd366a055530e78792821beb27a4f023374b3f20be486e8cc7d8caa7fe', '2021-08-18 08:41:32.000000',
        29, 48.3485, 4.5342, 465694, 24, '2021-08-18 08:44:36.390000', 256379, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611493068, 'a05e672f92a07256f8bacfbc56e7bd75fb4830404cfd064db1a38f8cb97a8415', '2021-08-18 08:19:01.000000',
        18, 50.0888, 3.2269, 227667, 0, '2021-08-18 08:19:11.167000', 168577, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611633843, 'ad07f549f4c6b4b8d563c812490760abcfd724b3e8428c49f003a230bb148e59', '2021-08-18 21:14:48.000000',
        13, 50.375, 3.4741, 506576, 89, '2021-08-18 21:14:50.478000', 165228, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611621993, 'c55dd59de9a3d502fcc043e8f1297e11a1747c33f094a692fd943b7d48c70cc5', '2021-08-18 18:31:43.000000',
        25, 48.6486, 2.5565, 629017, 0, '2021-08-18 18:31:45.151000', 326203, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611633952, 'ad07f549f4c6b4b8d563c812490760abcfd724b3e8428c49f003a230bb148e59', '2021-08-18 21:16:53.000000',
        15, 50.3465, 3.4811, 506579, 73, '2021-08-18 21:16:54.670000', 165230, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611523733, '4db9de89cb3de9a968b1437891d0255bfa824bc03bb9cc39d26a63c12f0c451c', '2021-08-18 10:28:28.000000',
        16, 50.9548, 5.2995, 25743, 0, '2021-08-18 10:28:33.299000', 16434, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611490079, '393dc9f1bd7ee63f36984f7ccb9f055528b1ad978432e1178beb631b9cfdc843', '2021-08-18 08:07:03.000000',
        15, 51.7735, 4.9531, 539100, 0, '2021-08-18 08:07:04.530000', 316977, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611703427, 'b2bd6671d1c5b7a28a6a1d0978adcd0feda137e55572d0c990702cfc366abcda', '2021-08-19 07:11:44.000000',
        13, 50.8085, 1.7341, 503525, 0, '2021-08-19 07:11:42.571000', 56800, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611479278, 'b58ad4676a7c324f74725dcf0abe372d4fed853213b90e96072c0f26a30f9334', '2021-08-18 07:24:15.000000',
        10, 47.8774, 2.2288, 246384, 0, '2021-08-18 07:24:15.700000', -1, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611482951, '92568237a4f298d8fed00c56f12ccc9a83f6bc1d5bbb7c5b58ec6255737c5d88', '2021-08-18 07:38:33.000000',
        23, 51.0332, 3.7524, 687645, 69, '2021-08-18 07:38:35.259000', 395121, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611503056, 'e0939a55640bf2b5503f50ad8d10e7619d4d683fa1719d4010f02fa726d90cf0', '2021-08-18 09:00:21.000000', 1,
        49.7109, 2.7727, 682077, 89, '2021-08-18 09:00:19.421000', 336015, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611503434, 'fe4478cc2c21faaa1cfb3a92afe99ff476e74f74f24fefea17c216aae72f4d8b', '2021-08-18 09:01:59.000000',
        32, 50.7966, 3.2193, 624141, 88, '2021-08-18 09:01:59.535000', 212147, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611558595, 'd9996891535ee83cb85fa51a33388adc12af4e65ddcc1854a8319e7436686302', '2021-08-18 12:57:06.000000', 1,
        49.1813, 4.1663, 681680, 44, '2021-08-18 12:57:07.343000', 407003, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611640025, '1d4013143770fa9a9b188b6ac647c77f446edccd30db7c144b47305d612d7ce9', '2021-08-18 23:04:30.000000',
        12, 47.03254, 4.86715, 241920, 80, '2021-08-18 23:04:33.631000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611567133, '0ce5ff98d38a737f1b9dbe90872616034e4389129ffd632958a737319b0b410b', '2021-08-18 13:32:25.000000',
        29, 50.3985, 3.113, 596550, 79, '2021-08-18 13:32:26.368000', 359983, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611653443, 'a0e3b862f265dc1aa0854e94bddbf12db579b79baa59a4d0998b592c2d7bfb53', '2021-08-19 02:34:16.000000',
        21, 50.2696, 3.32494, 273665, 0, '2021-08-19 02:35:29.463000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611637903, '6754b827800115d9ad0dde7f9131bf967e951791c48d9cf7572ac8d7e4692014', '2021-08-18 22:26:02.000000',
        19, 50.10202, 3.2051, 35166, 73, '2021-08-18 22:26:04.858000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611538270, '8abe33f13ca24de2825d5a60b43028425adfe078531afef4daa7445da2b65ee3', '2021-08-18 11:32:59.000000',
        12, 49.388547, 6.925795, 102083, 43, '2021-08-18 11:33:05.523000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607910420, 'c0b24bdbb008137026f8bc23ccd64abe5724230f900e84f54bf50936f671fcb8', '2021-07-28 08:49:37.000000',
        22, 52.1253, 4.4817, 75130, 84, '2021-07-28 08:49:39.583000', 43259, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611539151, '61976976b261153f92d6ac4abd754c6b9d782110bfba588f50a0e90ae3eea1f7', '2021-08-18 11:34:59.000000',
        19, 48.8045, 2.1306, 756844, 26, '2021-08-18 11:37:01.389000', 460670, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611492383, 'd0b7a010201358026f028983e8987e8f76f0ad13fb8e0828f4f8eedfa6a9bc06', '2021-08-18 08:16:24.000000',
        24, 50.6943, 2.8087, 532487, 0, '2021-08-18 08:16:28.508000', -1, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611621992, '800ea11604beb91b44526bb3e7af749c54b39168bf5609f9ce8f782d40b11db7', '2021-08-18 18:31:41.000000',
        24, 51.15432, 4.21677, 62275, 82, '2021-08-18 18:31:44.745000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611484528, '948d12497f61cc03c484686f60fa399a7db14ef7057514a9b92038b2078a1c86', '2021-08-18 07:45:02.000000', 7,
        50.847, 2.9055, 28382, 24, '2021-08-18 07:45:05.138000', 1831, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611493476, '7aaf32a4c072350adc03145ed85e1a1a821286a0502bbe23dfa48ddc46dd5a9c', '2021-08-18 08:20:58.000000',
        19, 48.6691, 4.1899, 47002, 76, '2021-08-18 08:20:59.258000', 27955, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611503853, 'd93557919f5c21389142bba2f0e8bb7009aadcabefbde31a6989c324b9418532', '2021-08-18 09:03:38.000000',
        30, 46.8109, 4.8513, 788236, 0, '2021-08-18 09:03:38.642000', 516100, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611503926, 'f27860b65d09934c8f719f3fcf5626efb2d6bfa66541c6dc164f9291c1037ae3', '2021-08-18 09:03:54.000000',
        30, 50.87009, 2.894373, -1, 0, '2021-08-18 09:03:57.289000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607951499, '45a037e96647f9462b201f4430d6e1c176a2a3c6ac672dcfc352ac0a29715b71', '2021-07-28 11:17:51.000000', 2,
        44.4029, 1.5233, 238499, 89, '2021-07-28 11:17:50.681000', 128585, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611480685, 'b03e8b30e475cbe6dd8024b70447224012136057a3ff2e5b710361aa5d9493b8', '2021-08-18 07:29:49.000000', 4,
        48.3302, 4.1152, 251797, 0, '2021-08-18 07:29:52.370000', 228150, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611549699, '5d59683715f4ebb7823c805e5cdcb90765209bed7afa09fbb0f2e9cae0cfa8da', '2021-08-18 12:21:33.000000',
        10, 45.44625, 10.89575, 4809, 4, '2021-08-18 12:21:35.418000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611634251, '5cff1dbb9d49b472e30dfad46de637e777e8cc50a21da8e6b2bc60129b4d38fe', '2021-08-18 21:22:03.000000',
        17, 50.8559, 2.7551, 646, 0, '2021-08-18 21:21:59.817000', 417, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607963608, '39a9a51a527845f54434103414ac6ff4b72c8b09e209124e7eec7bfa860e54ec', '2021-07-28 12:02:07.000000',
        20, 50.9472, 5.6996, 443829, 2, '2021-07-28 12:02:07.916000', 203522, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611497641, 'b23d4cb94f4e1aa8fcaeaff8b3983533e7d04a5dc8a6ba658f485a389c6e74c3', '2021-08-18 08:38:09.000000',
        17, 50.1848, 2.8736, 502737, 89, '2021-08-18 08:38:07.508000', 56402, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611698845, '1ee4fc53b753d18c30387d2fcdbcefc0ee3c58bbf0b069a77fcf0f76e3c2c667', '2021-08-19 06:51:49.000000',
        255, 52.2423, 6.5701, 41, 0, '2021-08-19 06:52:50.985000', 20100, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611566796, '3a14403175269eb7e05ffbef10cd4027d252676a9528023554d68961f087b384', '2021-08-18 13:30:54.000000', 0,
        50.4085, 3.0625, 117631, 24, '2021-08-18 13:30:54.466000', 72418, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611500238, 'eabf110fb39c19a9c3f6635a314e3f6e47e92befff6eb1fd13456b3f58a338f4', '2021-08-18 08:49:07.000000',
        29, 47.8986, 1.8744, 572247, 0, '2021-08-18 08:49:07.950000', 348662, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611544875, '2fb851364e98730b68a7b73a2f5717d7484dfbb049af92188875a5203dece7a2', '2021-08-18 12:01:49.000000',
        12, 51.0707, 6.3273, 711731, 0, '2021-08-18 12:01:52.835000', 71199, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611601300, '297e2bb800d965e0cef21521f2ef567f750c9b085a548c3e879889c8c4ef3fe0', '2021-08-18 16:05:51.000000',
        28, 48.58349, 2.94944, 317102, 70, '2021-08-18 16:05:53.001000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611638186, 'f88704644fed7cc6347b871f47367917d5e3ab9fcddd842594b03a0a605f986d', '2021-08-18 22:31:46.000000',
        10, 49.9119, 5.2664, 108773, 89, '2021-08-18 22:31:45.116000', 55095, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611548994, 'dd4329949a025d3198ed4dac360ca828d1a7291963651b93f9990e471950390e', '2021-08-18 12:18:56.000000',
        10, 51.1618, 3.1974, 141773, 89, '2021-08-18 12:18:56.199000', 97015, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611676312, 'a39c0ce21f4a3a7a20c1e4e348a5a7842fd2e9193796dee4803fb34897789bda', '2021-08-19 05:15:44.000000',
        21, 51.28457, 4.24128, 69159, 63, '2021-08-19 05:15:46.142000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611635340, '1d007e4d66843369d7f6d7350ecf975e0cb8dd29130d2ef5e110c79798ca2dfd', '2021-08-18 21:39:43.000000',
        26, 51.5024, 0.3625, 404650, 76, '2021-08-18 21:39:54.584000', 213669, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611680274, '147dfad8db5fb6131b67deb17b352f30fbb5cd4fd415390a2943b713c10328f3', '2021-08-19 05:33:06.000000',
        29, 51.0111, 2.9806, 1217185, 0, '2021-08-19 05:33:06.329000', 799225, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607910383, '9dbdce63acaa1e6dbbaa12fb1be61ef9bfe0d5b96a74ddb70d0dab76a0b0c7ea', '2021-07-28 08:49:28.000000',
        13, 48.1956, 4.949, 917942, 79, '2021-07-28 08:49:33.140000', 170685, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611503375, '2fb280b388c96d09af57b28d256cae682d72271f00f2a8a3e01b94606efd968e', '2021-08-18 09:01:41.000000', 1,
        47.4316, 9.7203, 209620, 79, '2021-08-18 09:01:43.407000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611591237, '99d94b914ffcb8c937c56fcf762be2a517fa014df1b63a25c726f87f376ad17b', '2021-08-18 15:16:54.000000',
        34, 50.9793, 2.6286, 253826, 0, '2021-08-18 15:16:56.651000', 184378, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611676440, '2d2a0f34c45d3a9c6354fa9836951f076716a4f25954d1b7609d4e3fbc72a4e5', '2021-08-19 05:16:19.000000', 6,
        50.3393, 3.4607, 599380, 87, '2021-08-19 05:16:27.729000', 321995, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611539625, '927acaf9b6f4b2033a22a62f3b23e5ff0fdfa2f64a27086712fd05c264664f9c', '2021-08-18 11:38:55.000000',
        33, 45.09461, 8.86814, 432008, 7, '2021-08-18 11:39:16.202000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611503690, 'a6651ffffa0cea4863a0e44fea3c44d346dba51124714d40644ab13d6e79b028', '2021-08-18 09:02:54.000000',
        18, 49.509, 2.7184, 393457, 88, '2021-08-18 09:02:56.413000', 237481, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611635950, '107f2c1d0eb1d8748ba58a716757667d5f82c8cef67231d54b53ff80b5a0eb2e', '2021-08-18 21:49:57.000000',
        31, 50.9114, 2.9212, 46924, 0, '2021-08-18 21:49:55.778000', 35048, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611703796, '0b132efb6a7a4b2aa1d3d562db7bf52775ef2b0520d25817eac8fb8efbb55571', '2021-08-19 07:09:27.000000', 7,
        43.33879, 5.3483, 319405, 24, '2021-08-19 07:13:16.589000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611544914, '99d94b914ffcb8c937c56fcf762be2a517fa014df1b63a25c726f87f376ad17b', '2021-08-18 12:01:58.000000',
        17, 50.9232, 2.908, 253725, 2, '2021-08-18 12:02:00.791000', 184299, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611481916, '65ce6e422d2f10260391cbec606621ed6a4dd8a3773a4e218ea751a2f607f491', '2021-08-18 07:34:30.000000', 9,
        50.4501, 1.6151, 262071, 0, '2021-08-18 07:34:32.944000', 114121, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611523377, '22164472a29862e8de2644dc4db4ae773170d793f8e2661fac900bfbe0c7413b', '2021-08-18 10:26:50.000000', 1,
        49.6374, 2.7577, 100760, 88, '2021-08-18 10:26:52.011000', 440825, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611529994, '22164472a29862e8de2644dc4db4ae773170d793f8e2661fac900bfbe0c7413b', '2021-08-18 10:56:39.000000', 1,
        50.0215, 2.8836, 100805, 90, '2021-08-18 10:56:40.926000', 440825, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611621014, '9d63ac057e70d27e5ab9687ff02467437d390bfdff17abaad8a953ee3625a307', '2021-08-18 18:21:34.000000',
        10, 51.0989, 5.0659, 502856, 88, '2021-08-18 18:21:34.910000', 290612, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611504924, 'f88704644fed7cc6347b871f47367917d5e3ab9fcddd842594b03a0a605f986d', '2021-08-18 09:07:29.000000',
        11, 52.031, 5.6039, 108275, 0, '2021-08-18 09:07:36.792000', 54842, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611490130, '49886bb2d99b7aa0374346decaded67d6333ce3d8332971f4f1a591dbbbc32d7', '2021-08-18 08:07:19.000000',
        27, 50.4279, 2.9948, 422546, 83, '2021-08-18 08:07:18.706000', 205640, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607960534, '50461fe666deb72f83cf84a411f60431039d8e5dded7628b2caa877673a8d9ee', '2021-07-28 11:50:44.000000',
        35, 50.2984, 3.6291, 506521, 68, '2021-07-28 11:50:44.990000', 243635, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611490459, '617e00c0be747e9d65d368edddeefc0a86cc104390ec0dcb7c5c3b65de4b480e', '2021-08-18 08:08:38.000000',
        20, 51.00867, 5.37028, -1, 34, '2021-08-18 08:08:41.033000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611500279, '10bb914a567debc59680cee0312f30522659de338a665ca6d06adf05115df26b', '2021-08-18 08:49:13.000000',
        10, 50.3412, 3.1026, 580389, 28, '2021-08-18 08:49:15.920000', 196608, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611646072, '490131889eaf8855235998d7115ee83765d442a7825e45ef30d46ebfa98c7617', '2021-08-19 00:53:23.000000', 4,
        50.3407, 2.93, 580987, 89, '2021-08-19 00:53:25.399000', 196608, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611508097, '0147e43ef72f8506049c642b6aaa3016d7db016690d6db3a71e161d6f4dd787c', '2021-08-18 09:20:25.000000', 3,
        51.39686, 4.6861, 723643, 30, '2021-08-18 09:20:24.465000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611493617, '7457032aa88da8e6163fb42374739f636fb536ab04de7e390d7e3c27e5e3b7ae', '2021-08-18 08:21:25.000000', 5,
        51.148003, 4.198028, 262894, 90, '2021-08-18 08:21:29.050000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611482605, 'b633f9f4b423c2b7583a71feb9e285b63849016caf39ab3a547a63be09f116c5', '2021-08-18 07:36:08.000000',
        33, 50.7183, 3.1958, 880807, 13, '2021-08-18 07:37:12.080000', 8641, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611648540, '9e79cbe0fbc944d5aaaa0de679b39c02c82654f039e5206c7ed102e5cd013048', '2021-08-19 01:31:31.000000', 1,
        50.9448, 3.1705, 253783, 0, '2021-08-19 01:31:33.067000', 192160, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611489686, '9fba6b8f0c91f7e1d4bae7464c0fffe5bc198479fc2e4dae4dd4a91d72328ab7', '2021-08-18 08:04:15.000000',
        11, 50.3684, 3.125, 787966, 16, '2021-08-18 08:05:20.116000', 395769, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611484359, '362b618263f6b80b16b81cd2dbad0f380eb830d64518f147675eee215992e6ef', '2021-08-18 07:44:21.000000',
        16, 47.8291, 3.5541, 224106, 75, '2021-08-18 07:44:20.566000', -1, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611527920, 'f5917f456756d31ea598588ba21a430657be19332cd0ef909bf7dae1a04b0429', '2021-08-18 10:46:56.000000',
        29, 50.2762, 2.8294, 150364, 28, '2021-08-18 10:47:25.345000', 84631, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611647991, '446e4555f468706790b54236fc3caa714e94ac0117c5977eccffd4348e80fc15', '2021-08-19 01:23:08.000000',
        21, 51.1187, 4.9026, 991196, 0, '2021-08-19 01:23:06.693000', 705174, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611624003, 'eabf110fb39c19a9c3f6635a314e3f6e47e92befff6eb1fd13456b3f58a338f4', '2021-08-18 18:52:41.000000',
        15, 50.1581, 3.1075, 572760, 94, '2021-08-18 18:52:41.073000', 348959, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611496312, '983766469d6f7a033fd49c017c3beb0f2ae0658888adaa9f27a945962bc67db7', '2021-08-18 08:32:47.000000',
        29, 51.1929, 4.4005, 282738, 46, '2021-08-18 08:32:45.127000', 165945, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611640008, '43fbc33fe69cee652b987b87ab5b76a2c9fa1bbff8b255c3b1b442f6b1452255', '2021-08-18 23:04:10.000000',
        30, 49.1699, 4.1787, 537816, 72, '2021-08-18 23:04:11.108000', -1, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1608727511, 'afb3666361da9719be4cfad99bef596e43f3c17c0b3ae0806d17534854a0599a', '2021-08-02 06:53:54.000000',
        24, 48.5505, 3.3082, 458673, 23, '2021-08-02 06:53:56.081000', 16630, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607955526, '5a00c5640f1dd065edbf9f345cef5c3e51cbd565fe156bd56fb85dce42c0a690', '2021-07-28 11:32:38.000000',
        19, 48.5017, 3.7538, 574955, 0, '2021-07-28 11:32:39.655000', 348344, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611539125, '19766ec6c1e68e279f054fde5a6b6cfcef5528115ff180fe61de39fef334bf55', '2021-08-18 11:36:53.000000',
        26, 50.83101, 6.30672, 22478, 72, '2021-08-18 11:36:55.653000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611538100, 'afad8ffabece80a1e127fcf71292b185c8c0bb01b1c81f5b9a84c33be2eaed35', '2021-08-18 11:32:11.000000',
        12, 50.8694, 2.8942, 364, 0, '2021-08-18 11:32:12.500000', -1, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611544650, '77c16f14ba90e88b6f566acb47fe3e07a632932e6fc2ca74178b11d996a3f239', '2021-08-18 11:58:54.000000',
        26, 50.9496, 3.1017, 582167, 0, '2021-08-18 12:00:58.013000', 358915, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611537828, 'e2cae8f713742866a13830767486ba1094c58a490769ecd5b48d4dfdb7a4875c', '2021-08-18 11:30:45.000000', 7,
        51.93245, 4.06404, 432778, 87, '2021-08-18 11:30:49.278000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1608726996, 'd910435ebdf7a6465722638a12368c3aeb0f4b5ef14ade7d6d117fb83c678b06', '2021-08-02 06:51:53.000000',
        31, 50.339908, 3.082507, 676359, 87, '2021-08-02 06:51:56.393000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611648578, '1dcc719f648fec6e2ab9b142dcba8bcf3ed5ac1db38ca32459fa3df54eed6c71', '2021-08-19 01:30:12.000000', 9,
        51.0968, 1.1519, 588740, 0, '2021-08-19 01:31:59.422000', -1, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611499618, 'e07e98d08683f3c721547912e0af4899875487afeefcf942fadbee6f8ad1e9b9', '2021-08-18 08:46:37.000000',
        33, 50.4201, 3.0297, 193293, 79, '2021-08-18 08:46:35.327000', 99954, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611529684, '28987f830055429917e21b11c355c501fb846f7e2d9af6f239e18e7ea053d637', '2021-08-18 10:55:12.000000',
        28, 48.8169, 2.3357, 836232, 69, '2021-08-18 10:55:12.476000', 537754, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611635442, 'fdbcd7dd6f616c47b54965d2ced5feaebc49b458fe8afd09d8cbf65608657dc2', '2021-08-18 21:41:20.000000',
        32, 48.2065, 1.8517, 525405, 0, '2021-08-18 21:41:21.394000', 265317, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611477023, '6d50a347f1ac0d7664955ee3510774496b0f1a10adfc97c8a22f12870321ed37', '2021-08-18 07:14:06.000000', 0,
        50.3912, 2.9822, 596483, 89, '2021-08-18 07:14:08.054000', -1, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1608727106, 'dd8b6df3224e60a44d803c802e687c271afabed31ad4ed5aa784fb4599bc60d2', '2021-08-02 06:51:11.000000',
        28, 47.13101, 5.43594, 139283, 0, '2021-08-02 06:52:24.111000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611648649, 'f18da38556329fe831267f8fe02bb042f494cca7012ee1846791b1b3e511eb4b', '2021-08-19 01:31:04.000000',
        20, 50.0889, 3.2266, 233833, 0, '2021-08-19 01:33:08.186000', 128360, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1608935858, 'f21e011f69b05b1fcf3856004ed29d2d4d81333995da258ea4b7509b539401b6', '2021-08-03 04:50:24.000000',
        35, 50.8957, 3.1767, 361230, 0, '2021-08-03 04:50:25.628000', 237325, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611500264, '9b19ae86a600d7b83a72c862fd0302c0afc385f6de09deed194deedd8f3e265a', '2021-08-18 08:49:12.000000',
        17, 49.1183, 6.1603, 921340, 68, '2021-08-18 08:49:13.779000', 172751, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611502267, '95f5e48a2650d21be04aaf852333cbf56155cd785b39617319a3c7e050bb9bb3', '2021-08-18 08:57:09.000000',
        13, 50.3958, 8.0798, 84303, 0, '2021-08-18 08:57:09.535000', 48267, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611653915, '1dcc719f648fec6e2ab9b142dcba8bcf3ed5ac1db38ca32459fa3df54eed6c71', '2021-08-19 02:41:03.000000', 3,
        50.9846, 2.2296, 588775, 81, '2021-08-19 02:41:05.264000', -1, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611481342, '71fbaf2b9480699e4d2cfdc2be086fd4436310c89b682a84331a1bf8057dec0c', '2021-08-18 07:32:16.000000',
        11, 53.5199, 10.04072, 432095, 0, '2021-08-18 07:32:18.696000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611601840, '95f5e48a2650d21be04aaf852333cbf56155cd785b39617319a3c7e050bb9bb3', '2021-08-18 16:09:07.000000',
        22, 50.6868, 6.0548, 84504, 90, '2021-08-18 16:09:06.937000', 48413, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611539041, 'b09e721992f802dd9e20bc158a15bc8c42acd43507b0c6bc5590a83faf263590', '2021-08-18 11:36:36.000000', 8,
        48.4958, 3.4899, 154120, 0, '2021-08-18 11:36:37.141000', 85026, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607910815, '439b8965f1b67f37db45b5a49fd19c02c9748389276f686ceab39b07ec691041', '2021-07-28 08:50:07.000000',
        24, 50.37, 3.0377, 61547, 0, '2021-07-28 08:51:09.465000', 35848, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611638043, 'f88704644fed7cc6347b871f47367917d5e3ab9fcddd842594b03a0a605f986d', '2021-08-18 22:29:16.000000',
        17, 49.9353, 5.2356, 108769, 88, '2021-08-18 22:29:14.887000', 55092, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607963614, '7bf68e3d02eb9da9068e26322bac2ca447c101f5b5be081ddf6a86b3d4107967', '2021-07-28 12:02:01.000000',
        27, 50.6169, 3.08078, 315167, 59, '2021-07-28 12:02:09.295000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611512205, '93665d979a6a199e5e6369da03dae3fe3f580e61fc6c4734de0c3ac86167773a', '2021-08-18 09:37:44.000000',
        29, 50.5742, 3.0303, 613794, 4, '2021-08-18 09:37:45.746000', 336778, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611503838, 'db8c138b5bfb1679aebbe91c0cf1fcefaf7071dce0fdcdc1f0ac62603b526b8e', '2021-08-18 09:03:32.000000',
        16, 50.9733, 4.4896, 596448, 87, '2021-08-18 09:03:34.071000', 330848, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611539040, '9a4923cb775465b43fd53390187b7455fa68038fc04684ab474911cb28b4d7b2', '2021-08-18 11:36:37.000000',
        25, 51.0212, 3.0436, 192512, 0, '2021-08-18 11:36:37.016000', 110191, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611611704, 'cf437ddbad6970f5e8a1ba0dd9efd07e7ac6787c845fa50c2246eae1f6330e29', '2021-08-18 17:06:32.000000',
        12, 50.45, 3.0153, 125461, 0, '2021-08-18 17:06:38.912000', -1, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611532460, '38e3cdf925f53817fa97d61178e857aa1a5075f4df7946f0d75a13613069fc2c', '2021-08-18 11:07:15.000000',
        34, 50.6728, 3.2312, 315003, 0, '2021-08-18 11:07:17.557000', 181191, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611523698, '77286d0bf16a5c6bdfff5919812496dbc90be393615bba32730239ec4573d20f', '2021-08-18 10:28:19.000000', 7,
        46.343, 6.9333, 500810, 31, '2021-08-18 10:28:20.212000', 308278, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611566967, '6bda72296babf9b29bffe834f7d77ae618b512863d03106690fd992119ed4ca7', '2021-08-18 13:31:38.000000',
        28, 48.2372, 4.4514, 559866, 29, '2021-08-18 13:31:40.714000', 345809, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1608727411, '70b90549b4ae13976008941b30b781a2e92638d7488b97f9eb12f0d3a4e99f54', '2021-08-02 06:51:27.000000', 7,
        50.8736, 6.0355, 272675, 0, '2021-08-02 06:53:29.937000', 136237, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611521516, '35006e79f4b1d2171f9d3021039a37af06fcbd0844bf72f2a2a28ea410b448f9', '2021-08-18 10:18:24.000000',
        18, 52.863347, 10.492812, 180418, 67, '2021-08-18 10:18:26.779000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611621733, '251ffbf1c3a0440f9132f3f7217e4308702e27137427891c34839ac30c742de9', '2021-08-18 18:29:06.000000',
        33, 52.83342, 8.4427, 156528, 0, '2021-08-18 18:29:11.073000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611698829, '726e34e19af1adec5032108f9fd6d0443c15b6a699c477be40b97c2a43230b63', '2021-08-19 06:52:43.000000', 0,
        49.9147, 2.8507, 488262, 89, '2021-08-19 06:52:44.917000', 243383, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611523686, '676db79df7a41f1497d23e9ed037c6084cfe588b7b5507ce9408228b26b64c2c', '2021-08-18 10:24:17.000000',
        30, 51.2956, 4.2531, 265340, 0, '2021-08-18 10:28:18.296000', 155224, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611635310, 'a5a7851d0f93ed76b6c85f0680ffc5f58bcd7319ac7d20f06329a0bf33850095', '2021-08-18 21:39:18.000000',
        25, 50.9603, 2.2079, 694272, 89, '2021-08-18 21:39:16.224000', 367941, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611620982, 'c2604752ff3cad04713fb30193590e761bdb24d5145d080cc793c01bcf74a582', '2021-08-18 18:21:24.000000',
        22, 48.2351, 4.4507, 581974, 0, '2021-08-18 18:21:24.343000', 353065, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611513450, '24e14ad73853e2263c94ff6f655986c3b756ffa7215a99afeb0d8cb7c3eb3314', '2021-08-18 09:43:06.000000', 4,
        50.4447, 2.9771, 143001, 0, '2021-08-18 09:43:22.923000', -1, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611508803, '59fbb4e5dd0dc7c38cd425451279f7ba33e6840172c8f88dc416ac45a1b08f28', '2021-08-18 09:23:30.000000', 4,
        49.2413, 2.6667, 297278, 89, '2021-08-18 09:23:29.016000', 178552, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611539055, 'dd4329949a025d3198ed4dac360ca828d1a7291963651b93f9990e471950390e', '2021-08-18 11:36:43.000000',
        35, 51.0109, 2.6827, 141723, 65, '2021-08-18 11:36:41.688000', 96993, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611503021, 'c9e33951a430606550e5ea65bc3a20b7972eae5962ff14bc176b68210bd14bcc', '2021-08-18 09:00:08.000000',
        29, 50.947735, 3.110842, -1, 0, '2021-08-18 09:00:11.628000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607951212, 'bf5e15f9ca51de4245ab0e4ad761c4f1327d8793b71f847bc10321dcaec0d21c', '2021-07-28 11:16:34.000000',
        28, 53.512962, 9.979913, 451416, 0, '2021-07-28 11:16:36.534000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611658628, 'c9df1227dcf24d53ba71a94cdcfdef4470b1939e9348aa01f3f00f493ba3dc64', '2021-08-19 03:28:15.000000',
        21, 50.3899, 2.832, 384200, 29, '2021-08-19 03:28:18.023000', 69312, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611634104, '854ba69cec35f6f3a7f04eb1b9cc091504d6e0119d998583104f49f3ed55ee27', '2021-08-18 21:19:20.000000',
        19, 48.5039, 4.1665, 497707, 89, '2021-08-18 21:19:22.997000', 20642, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611494045, '61013e4878aa5c0861439fd001c755ba702b8d4e91dcadfa8ecfb7c08f4155bf', '2021-08-18 08:23:05.000000', 0,
        50.743142, 3.292587, -1, 0, '2021-08-18 08:23:25.283000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611703784, '22af166bbf9e9c940199929b34c73b8eca3dbbda61a540684141433ce5f40be9', '2021-08-19 07:13:14.000000',
        10, 50.9724, 7.542, 295402, 79, '2021-08-19 07:13:14.120000', 167014, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611648684, '35ca794fd792f6a99632327e9972b2db8d095cfe9f65877e0e329a44c623ffc6', '2021-08-19 01:33:40.000000', 6,
        45.904, 4.8281, 871400, 88, '2021-08-19 01:33:41.156000', 434653, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611601218, '3a14403175269eb7e05ffbef10cd4027d252676a9528023554d68961f087b384', '2021-08-18 16:05:26.000000',
        27, 50.3709, 3.1104, 117647, 0, '2021-08-18 16:05:26.371000', 72434, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611602799, 'f1f8a5f50fe9558d864abc0022662687d61444d197481048f81f7383dadbe898', '2021-08-18 16:14:26.000000', 0,
        50.2699, 3.3254, 758835, 11, '2021-08-18 16:14:31.014000', 153929, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611567730, '125d1f0fec42ecbe043e4559b2d31984f302154eb27b81b549be696ef12b4297', '2021-08-18 13:34:03.000000',
        14, 51.7528, 8.3877, 422932, 0, '2021-08-18 13:35:02.801000', 237019, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611479630, '5d1365ed29b545dece43eff54b24c57d32a4c9e814a6b748f65efc142405b136', '2021-08-18 07:25:37.000000',
        28, 51.2912, 4.3683, 365838, 87, '2021-08-18 07:25:36.821000', 221286, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611508745, 'f0ddaf068fcb477673a9ecfa5f271d8789f9b7c16b855b2534b73df413247386', '2021-08-18 09:23:15.000000', 9,
        51.1203, 3.0817, 253563, 38, '2021-08-18 09:23:15.357000', 191999, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611490091, 'dacde31028852160de24f2c54756be54454620a6628e9716f172a1d4a8a35c41', '2021-08-18 08:07:05.000000',
        25, 48.56524, 1.97585, 1049307, 69, '2021-08-18 08:07:08.093000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611513316, 'db8c138b5bfb1679aebbe91c0cf1fcefaf7071dce0fdcdc1f0ac62603b526b8e', '2021-08-18 09:42:46.000000',
        14, 50.6009, 4.7896, 596503, 88, '2021-08-18 09:42:48.878000', 330874, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611566735, '05c866204b2841adf616be606926b718e5f73b23a02b27aebfe39b97a39c36ff', '2021-08-18 13:30:31.000000',
        19, 43.686, 10.35647, 171610, 53, '2021-08-18 13:30:35.010000', 0, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611521374, 'f0ddaf068fcb477673a9ecfa5f271d8789f9b7c16b855b2534b73df413247386', '2021-08-18 10:17:54.000000', 4,
        51.1345, 3.1055, 253566, 5, '2021-08-18 10:17:54.711000', 192003, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611539291, '02b25ab5bd9edba84a05a2b49e940e1d331846482ef4300f3168503b183ae617', '2021-08-18 11:37:36.000000',
        22, 50.7188, 3.283, 21850, 13, '2021-08-18 11:37:38.419000', 1361, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1608643523, '67cad0c32e213e0ec98a098956240a94b6f21f7489e591d7dc1d2d5fc2b6959e', '2021-08-01 09:49:17.000000',
        29, 48.2351, 4.4499, 41130, 0, '2021-08-01 09:49:15.575000', 24331, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611566835, '28987f830055429917e21b11c355c501fb846f7e2d9af6f239e18e7ea053d637', '2021-08-18 13:31:06.000000',
        27, 48.8167, 2.3402, 836262, 69, '2021-08-18 13:31:06.177000', 537774, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611536353, '4451f0e6227dedc4431a7cd5ec02c47ec134695a7997522009ca3c3e407b1070', '2021-08-18 11:23:57.000000', 2,
        50.733837, 3.127763, 613350, 87, '2021-08-18 11:23:59.598000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607910917, '71045cfac6b72a19977fe03ad0bb3a717b579e4340a8aa48339c57e693f548ff', '2021-07-28 08:51:32.000000',
        12, 50.3677, 3.1263, 581462, 10, '2021-07-28 08:51:33.411000', 275047, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611520825, '6d50a347f1ac0d7664955ee3510774496b0f1a10adfc97c8a22f12870321ed37', '2021-08-18 10:13:31.000000', 4,
        50.3951, 3.1013, 596504, 0, '2021-08-18 10:15:33.592000', -1, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611503458, '22164472a29862e8de2644dc4db4ae773170d793f8e2661fac900bfbe0c7413b', '2021-08-18 09:01:03.000000',
        10, 48.8114, 2.3284, 100652, 10, '2021-08-18 09:02:04.364000', 440825, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611601032, '06f015b5f8f6e1443fe34785a27da8cf098d63d165afd51965d61fe0f954e883', '2021-08-18 16:04:25.000000',
        20, 53.4792, 8.6178, 422426, 85, '2021-08-18 16:04:26.750000', 234082, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611508108, '34f9f0b51ba7955dfa6608891f9852adde5310a87c00c06d8eb4fcb037d15d05', '2021-08-18 09:20:28.000000', 7,
        50.3403, 3.0231, 728357, 53, '2021-08-18 09:20:28.831000', 159785, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611539115, '82cdfceaa96c139b40f13152b73daa2179a3b036ca232c9f21a7ae33239c2366', '2021-08-18 11:33:33.000000',
        31, 50.8366, 6.424, 274562, 0, '2021-08-18 11:36:53.659000', 153753, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611637893, '5a0e1704b3a0bc22066c7858644db0de97a04036390f017e0c733273a2eea945', '2021-08-18 22:25:00.000000', 0,
        48.643921, 9.428548, 515513, 0, '2021-08-18 22:25:50.097000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611680107, '578d80702c2f8a7c105e77d502b520c08c9a4d10d0611666e4e5cadb6536cab9', '2021-08-19 05:32:17.000000', 0,
        50.7148, 3.1238, 200003, 88, '2021-08-19 05:32:19.955000', 70914, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611503852, 'fdcb9d1e2c8ae61b559afe278cfe2a854f77a5d247014994918a708f8a760956', '2021-08-18 09:03:35.000000', 0,
        50.8406, 2.8763, 905, 0, '2021-08-18 09:03:38.228000', -1, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611503681, '6bda72296babf9b29bffe834f7d77ae618b512863d03106690fd992119ed4ca7', '2021-08-18 09:02:52.000000', 7,
        48.3296, 4.1251, 559805, 50, '2021-08-18 09:02:53.569000', 345769, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611493061, 'f78fd3c4b2011b9cef9f25109dbda3e4c0ffb2ea09d634d92a783e836f66e716', '2021-08-18 08:19:08.000000',
        17, 49.8552, 3.261, 599122, 22, '2021-08-18 08:19:09.790000', 321859, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1608649123, '2842a6e2c583e43c16d1afc2051b67e26f477c5241a4a4dbd56376a3485a0d3b', '2021-08-01 13:27:14.000000', 4,
        49.53174, 8.41879, 291593, 33, '2021-08-01 13:27:16.269000', 0, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611512216, 'd9996891535ee83cb85fa51a33388adc12af4e65ddcc1854a8319e7436686302', '2021-08-18 09:30:47.000000',
        28, 48.2352, 4.4521, 681553, 0, '2021-08-18 09:37:48.933000', 406913, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611512143, '1b067ff150bfb37eb08248fee538191e8d3318e4213715a9681e9049ee69885d', '2021-08-18 09:37:27.000000', 1,
        51.082, 3.6594, 622336, 82, '2021-08-18 09:37:29.223000', -1, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611635279, 'ad07f549f4c6b4b8d563c812490760abcfd724b3e8428c49f003a230bb148e59', '2021-08-18 21:38:41.000000',
        20, 50.3712, 3.6102, 506593, 0, '2021-08-18 21:38:45.430000', 165236, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607951238, 'c687273aa521adafd0ed32d6776b6c82f045399cfca272d1b6c05ca5dc464e1e', '2021-07-28 11:16:46.000000', 5,
        51.1788, 4.2752, 541298, 90, '2021-07-28 11:16:45.032000', 227813, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611478276, '7dfb4d53f13d8af53b94043311075a6fba38aa2ac2ead29f8f029136c49a2793', '2021-08-18 07:19:31.000000',
        33, 47.2285, 4.5871, 465076, 89, '2021-08-18 07:19:32.574000', 20295, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611623253, 'a421a2a306e5249281847262b2e18505ea2a9c1d18fe5db0fff4da6ac998b07f', '2021-08-18 18:44:12.000000', 5,
        48.9916, 2.6195, 468097, 84, '2021-08-18 18:44:13.141000', 235396, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1608727419, '6934cbf19e2e08082b12c48f4b1b788b0ac73e8ae5dd86e97c3723c29698bacd', '2021-08-02 06:46:49.000000',
        29, 48.2219, 4.1315, 244296, 0, '2021-08-02 06:53:32.422000', 141795, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611638865, '734a8eb54f88ed21909b48065b1c67de9f2cbcb466398474f5c5611ef7aed52e', '2021-08-18 22:42:07.000000',
        255, 48.7994, 1.9525, 627761, 2, '2021-08-18 22:43:10.007000', 369820, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611638119, 'a2130fca8ee5a1779bcbcf78e6a08f8e5393a31452de2b1ce6ab8e65e3f8ff08', '2021-08-18 22:30:25.000000',
        15, 50.869, 2.894, 22, 0, '2021-08-18 22:30:25.703000', -1, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611513308, 'f6a276da9f90bdeb881796f939022bda10be6b1aac347c74003f5b91f0583db9', '2021-08-18 09:42:43.000000',
        28, 50.9455, 3.0996, 548940, 0, '2021-08-18 09:42:46.220000', 348999, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611635319, 'f1136eab4d12d03deaf7f3cc5d30717d3896738cdf5375b7345c02c43575ab71', '2021-08-18 21:39:24.000000', 6,
        51.7734, 4.953, 280070, 0, '2021-08-18 21:39:25.227000', 149894, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611638845, 'dd269314535a453a31a59299b238d9731ccf381f92836a53b2d95b2c9ce9d837', '2021-08-18 22:42:55.000000',
        30, 50.912, 6.8797, 1299379, 89, '2021-08-18 22:42:54.972000', 716299, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607957958, 'c59353ad9a7c62bf126a773b2d096bb35eeec8ec705bdfd51f73aa633f1ec171', '2021-07-28 11:41:26.000000',
        30, 47.375, 4.3901, 241242, 88, '2021-07-28 11:41:25.792000', -1, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611513287, 'd327ef1eba3a4ade469d98c5ca8cf5c45f45f3eaeabfc6beb0e65730a3947364', '2021-08-18 09:42:31.000000',
        13, 50.9005, 5.456, 763504, 0, '2021-08-18 09:42:40.858000', 689392, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611621644, '734a8eb54f88ed21909b48065b1c67de9f2cbcb466398474f5c5611ef7aed52e', '2021-08-18 18:28:02.000000',
        18, 49.5087, 2.7182, 627603, 90, '2021-08-18 18:28:02.171000', 369742, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611539129, '887bf291189663bd9d365a8e78e306b97b0d89839938342e1827323578c8cadc', '2021-08-18 11:36:55.000000',
        26, 51.5837, 7.4041, 96557, 79, '2021-08-18 11:36:56.387000', 53423, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611638846, '22164472a29862e8de2644dc4db4ae773170d793f8e2661fac900bfbe0c7413b', '2021-08-18 22:42:54.000000', 5,
        50.9268, 3.1707, 100923, 0, '2021-08-18 22:42:55.145000', 440825, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611648768, '4384f6b70c7fe5a2bc40f65a68327598813a1554fd51771cd18810d791768401', '2021-08-19 01:35:01.000000',
        32, 50.8137, 2.0851, 468388, 46, '2021-08-19 01:35:02.641000', 235533, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1607438232, '887c16dd6dbb9d3872d335974a6860a6ffde23ab3cf2dd49ec7decb8bc45a2b9', '2021-07-26 11:21:16.000000', 3,
        50.4465, 3.0017, 123262, 66, '2021-07-26 11:21:18.104000', -1, null, '6355f4d9-6361-4a55-89d8-57bcd5666532');
INSERT INTO public.data_row (id, dayid, gps_time, heading, latitude, longitude, odometer, speed, timestamp,
                             total_fuel_used, dataset, offer)
VALUES (1611634882, 'f6a276da9f90bdeb881796f939022bda10be6b1aac347c74003f5b91f0583db9', '2021-08-18 21:25:28.000000',
        28, 51.4965, 7.3966, 549440, 84, '2021-08-18 21:32:15.441000', 349289, null,
        '6355f4d9-6361-4a55-89d8-57bcd5666532');

SELECT setval('hibernate_sequence', 4);
